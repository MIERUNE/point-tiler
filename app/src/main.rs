use std::collections::HashMap;
use std::ffi::OsStr;
use std::io::Write;
use std::{
    fs,
    path::{Path, PathBuf},
};

use chrono::Local;
use env_logger::Builder;
use glob::glob;
use log::LevelFilter;

use pcd_core::pointcloud::{
    decimation::decimator::{PointCloudDecimator, VoxelDecimator},
    point::{Point, PointCloud},
};
use pcd_exporter::tiling;
use pcd_exporter::{
    cesiumtiles::{make_tile_content, pointcloud_to_tiles},
    gltf::generate_quantized_glb,
    tiling::{geometric_error, TileContent, TileTree},
};
use pcd_parser::parsers::csv::CsvParserProvider;
use pcd_parser::parsers::{get_extension, Extension};
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};
use pcd_transformer::{
    builder::PointCloudTransformBuilder, runner::PointCloudTransformer, Transformer,
};
use projection_transform::cartesian::geodetic_to_geocentric;

use clap::Parser;
use rayon::iter::{IntoParallelIterator as _, ParallelIterator as _};

#[derive(Parser, Debug)]
#[command(
    name = "Point Tiler",
    about = "A tool for converting point cloud data into 3D Tiles",
    author = "MIERUNE Inc.",
    version = "0.0.1"
)]
struct Cli {
    #[arg(short, long, required = true, num_args = 1.., value_name = "FILE")]
    input: Vec<String>,

    #[arg(short, long, required = true, value_name = "DIR")]
    output: String,

    #[arg(short, long, required = true)]
    epsg: u16,

    #[arg(long, default_value_t = 15)]
    min: u8,

    #[arg(long, default_value_t = 18)]
    max: u8,
}

fn check_and_get_extension(paths: &[PathBuf]) -> Result<Extension, String> {
    let mut extensions = vec![];
    for path in paths.iter() {
        let extension = path.extension().and_then(OsStr::to_str);
        match extension {
            Some(ext) => extensions.push(ext),
            None => return Err("File extension is not found".to_string()),
        }
    }
    extensions.sort();
    extensions.dedup();

    if extensions.len() > 1 {
        return Err("Multiple extensions are not supported".to_string());
    }

    Ok(get_extension(extensions[0]))
}

fn expand_globs(input_patterns: Vec<String>) -> Vec<PathBuf> {
    let mut paths = Vec::new();
    for pattern in input_patterns {
        if pattern.contains('*') || pattern.contains('?') || pattern.contains('[') {
            for entry in glob(&pattern).expect("Failed to read glob pattern") {
                match entry {
                    Ok(path) => paths.push(path),
                    Err(e) => eprintln!("Error: {:?}", e),
                }
            }
        } else {
            paths.push(PathBuf::from(pattern));
        }
    }
    paths
}

fn main() {
    Builder::new()
        .format(|buf, record| {
            writeln!(
                buf,
                "{} [{}] - {}",
                Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.args()
            )
        })
        .filter(None, LevelFilter::Info)
        .init();

    let args = Cli::parse();

    log::info!("input files: {:?}", args.input);
    log::info!("output folder: {}", args.output);
    log::info!("input EPSG: {}", args.epsg);
    log::info!("min zoom: {}", args.min);
    log::info!("max zoom: {}", args.max);

    let start = std::time::Instant::now();

    log::info!("start processing...");
    // let input_files = args.input.iter().map(PathBuf::from).collect::<Vec<_>>();
    let input_files = expand_globs(args.input);
    log::info!("Expanded input files: {:?}", input_files);

    let output_path = PathBuf::from(args.output);
    std::fs::create_dir_all(&output_path).unwrap();

    log::info!("start parsing...");
    let start_local = std::time::Instant::now();

    let extension = check_and_get_extension(&input_files).unwrap();
    let parser = match extension {
        Extension::Las => {
            let las_parser_provider = LasParserProvider {
                filenames: input_files,
                epsg: args.epsg,
            };
            let provider = las_parser_provider;
            provider.get_parser()
        }
        Extension::Laz => {
            let las_parser_provider = LasParserProvider {
                filenames: input_files,
                epsg: args.epsg,
            };
            let provider = las_parser_provider;
            provider.get_parser()
        }
        Extension::Csv => {
            let csv_parser_provider = CsvParserProvider {
                filenames: input_files,
                epsg: args.epsg,
            };
            let provider = csv_parser_provider;
            provider.get_parser()
        }
        Extension::Txt => {
            let csv_parser_provider = CsvParserProvider {
                filenames: input_files,
                epsg: args.epsg,
            };
            let provider = csv_parser_provider;
            provider.get_parser()
        }
    };
    // TODO: Allow each chunk to be retrieved
    let point_cloud = match parser.parse() {
        Ok(point_cloud) => point_cloud,
        Err(e) => {
            log::error!("Failed to parse point cloud: {:?}", e);
            return;
        }
    };
    // let point_cloud = parser.parse();
    log::info!("finish parsing in {:?}", start_local.elapsed());

    log::info!("start transforming...");
    let output_epsg = 4979;
    let start_local = std::time::Instant::now();
    let transform_builder = PointCloudTransformBuilder::new(output_epsg);
    let transformer = PointCloudTransformer::new(Box::new(transform_builder));

    let transformed = transformer.execute(point_cloud.clone());
    log::info!("Finish transforming in {:?}", start_local.elapsed());

    let min_zoom = args.min;
    let max_zoom = args.max;

    log::info!("start tiling...");
    let start_local = std::time::Instant::now();
    let epsg = &transformed.metadata.epsg;
    let mut tile_point_clouds: HashMap<(u8, u32, u32), Vec<Point>> = HashMap::new();

    for point in &transformed.points {
        let tile_coords = tiling::scheme::zxy_from_lng_lat(max_zoom, point.x, point.y);
        tile_point_clouds
            .entry(tile_coords)
            .or_default()
            .push(point.clone());
    }

    for z in (min_zoom..max_zoom).rev() {
        let mut parent_tiles: HashMap<(u8, u32, u32), Vec<Point>> = HashMap::new();

        for (&(child_z, child_x, child_y), points) in &tile_point_clouds {
            if child_z == z + 1 {
                let parent_x = child_x / 2;
                let parent_y = child_y / 2;
                let parent_coords = (z, parent_x, parent_y);

                parent_tiles
                    .entry(parent_coords)
                    .or_default()
                    .extend(points.iter().cloned());
            }
        }

        for (coords, pts) in parent_tiles {
            tile_point_clouds.entry(coords).or_default().extend(pts);
        }
    }

    let mut tiled_pointcloud = Vec::new();
    for ((z, x, y), points) in tile_point_clouds {
        if z >= min_zoom && z <= max_zoom {
            let tile_pointcloud = PointCloud::new(points, *epsg);
            tiled_pointcloud.push(((z, x, y), tile_pointcloud));
        }
    }
    log::info!("Finish tiling in {:?}", start_local.elapsed());

    log::info!("start exporting tiles...");
    let start_local = std::time::Instant::now();
    let mut tile_contents: Vec<TileContent> = tiled_pointcloud
        .into_par_iter()
        .map(|(tile_coords, pointcloud)| {
            let tile_content = make_tile_content(&tile_coords, &pointcloud);

            let mut points = vec![];
            let ellipsoid = projection_transform::ellipsoid::wgs84();

            for point in pointcloud.iter() {
                let (lng, lat, height) = (point.0, point.1, point.2);
                let (x, y, z) = geodetic_to_geocentric(&ellipsoid, lng, lat, height);
                points.push(Point {
                    x,
                    y: z,
                    z: -y,
                    color: point.3.color.clone(),
                    attributes: point.3.attributes.clone(),
                });
            }
            let transformed = PointCloud::new(points, output_epsg);

            let geometric_error = geometric_error(tile_coords.0, tile_coords.2);

            let voxel_size = geometric_error * 0.1;

            let decimetor = VoxelDecimator { voxel_size };
            let decimated_points = decimetor.decimate(&transformed.points);
            let decimated = PointCloud::new(decimated_points, output_epsg);

            let glb_path = format!(
                "{}/{}",
                output_path.to_string_lossy(),
                tile_content.content_path
            );
            log::info!("write GLB: {:?}", glb_path);
            std::fs::create_dir_all(std::path::Path::new(&glb_path).parent().unwrap()).unwrap();

            let glb = generate_quantized_glb(decimated).unwrap();

            let writer = std::fs::File::create(glb_path).unwrap();
            let _ = glb.to_writer_with_alignment(writer, 8);

            tile_content
        })
        .collect();
    log::info!("Finish exporting tiles in {:?}", start_local.elapsed());

    let mut tree = TileTree::default();
    for content in tile_contents.drain(..) {
        tree.add_content(content);
    }

    let tileset = cesiumtiles::tileset::Tileset {
        asset: cesiumtiles::tileset::Asset {
            version: "1.1".to_string(),
            ..Default::default()
        },
        root: tree.into_tileset_root(),
        geometric_error: 1e+100,
        ..Default::default()
    };

    let root_tileset_path = output_path.join(Path::new("tileset.json"));
    log::info!("write tileset.json: {:?}", root_tileset_path);
    fs::create_dir_all(root_tileset_path.parent().unwrap()).unwrap();
    fs::write(
        root_tileset_path,
        serde_json::to_string_pretty(&tileset).unwrap(),
    )
    .unwrap();

    log::info!("Elapsed: {:?}", start.elapsed());

    log::info!("Finish processing");
}
