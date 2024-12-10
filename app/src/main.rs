use std::collections::HashMap;
use std::ffi::OsStr;
use std::fs::{File, OpenOptions};
use std::io::{BufRead as _, BufReader, BufWriter, Write};
use std::{
    fs,
    path::{Path, PathBuf},
};

use chrono::Local;
use clap::Parser;
use env_logger::Builder;
use glob::glob;
use log::LevelFilter;
use rayon::iter::{IntoParallelRefIterator as _, ParallelIterator as _};
use tempfile::tempdir;

use pcd_core::pointcloud::{
    decimation::decimator::{PointCloudDecimator, VoxelDecimator},
    point::{Point, PointCloud},
};
use pcd_exporter::tiling;
use pcd_exporter::{
    cesiumtiles::make_tile_content,
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

fn write_points_to_tile(
    dir_path: &Path,
    tile: (u8, u32, u32),
    points: &[Point],
) -> std::io::Result<()> {
    let (z, x, y) = tile;
    let tile_path = dir_path.join(format!("{}/{}/{}.bin", z, x, y));

    fs::create_dir_all(tile_path.parent().unwrap())?;

    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&tile_path)?;

    let mut writer = BufWriter::new(file);

    for p in points {
        let line = serde_json::to_string(p).unwrap();
        writeln!(writer, "{}", line)?;
    }

    Ok(())
}

fn read_points_from_tile(file_path: &Path) -> std::io::Result<Vec<Point>> {
    let file = File::open(file_path)?;
    let reader = BufReader::new(file);

    let mut points = Vec::new();
    for line in reader.lines() {
        let line = line?;
        let p: Point = serde_json::from_str(&line).unwrap();
        points.push(p);
    }

    Ok(points)
}

fn extract_tile_coords(file_path: &Path) -> (u8, u32, u32) {
    let x_dir = file_path.parent().unwrap();
    let z_dir = x_dir.parent().unwrap();

    let z: u8 = z_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let x: u32 = x_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let y: u32 = file_path
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();

    (z, x, y)
}

fn get_tile_list_for_zoom(base_path: &Path, z: u8) -> Vec<PathBuf> {
    let pattern = base_path.join(format!("{}/**/*.bin", z));
    let mut files = Vec::new();
    files.extend(
        glob::glob(pattern.to_str().unwrap())
            .unwrap()
            .filter_map(Result::ok),
    );
    files
}

fn aggregate_zoom_level(base_path: &Path, z: u8) -> std::io::Result<()> {
    let child_z = z + 1;
    let child_files = get_tile_list_for_zoom(base_path, child_z);

    for child_file in child_files {
        let (cz, cx, cy) = extract_tile_coords(&child_file);
        assert_eq!(cz, child_z);

        let file = File::open(&child_file)?;
        let reader = BufReader::new(file);

        let mut parent_map: HashMap<(u8, u32, u32), Vec<Point>> = HashMap::new();

        for line in reader.lines() {
            let line = line?;
            let p: Point = serde_json::from_str(&line).unwrap();

            let parent_x = cx / 2;
            let parent_y = cy / 2;
            let parent_coords = (z, parent_x, parent_y);
            parent_map.entry(parent_coords).or_default().push(p);
        }

        for (parent_tile, pts) in parent_map {
            write_points_to_tile(base_path, parent_tile, &pts)?;
        }
    }

    Ok(())
}

fn export_tiles_to_glb(
    base_path: &Path,
    output_path: &Path,
    min_zoom: u8,
    max_zoom: u8,
) -> std::io::Result<Vec<TileContent>> {
    let mut all_tiles = Vec::new();
    for z in min_zoom..=max_zoom {
        let files = get_tile_list_for_zoom(base_path, z);
        all_tiles.extend(files);
    }

    let tile_contents: Vec<TileContent> = all_tiles
        .par_iter()
        .map(|tile_file| {
            let (tz, tx, ty) = extract_tile_coords(tile_file);
            let points = read_points_from_tile(tile_file).unwrap();
            let epsg = 4979;
            let pc = PointCloud::new(points, epsg);

            let tile_content = make_tile_content(&(tz, tx, ty), &pc);

            let ellipsoid = projection_transform::ellipsoid::wgs84();
            let transformed_points: Vec<Point> = pc
                .points
                .par_iter()
                .map(|orig| {
                    let (lng, lat, height) = (orig.x, orig.y, orig.z);
                    let (xx, yy, zz) = geodetic_to_geocentric(&ellipsoid, lng, lat, height);
                    Point {
                        x: xx,
                        y: zz,
                        z: -yy,
                        color: orig.color.clone(),
                        attributes: orig.attributes.clone(),
                    }
                })
                .collect();

            let transformed_pc = PointCloud::new(transformed_points, epsg);
            let geometric_error_value = geometric_error(tz, ty);
            let voxel_size = geometric_error_value * 0.1;
            let decimator = VoxelDecimator { voxel_size };
            let decimated_points = decimator.decimate(&transformed_pc.points);
            let decimated = PointCloud::new(decimated_points, epsg);

            let glb_path = output_path.join(&tile_content.content_path);
            fs::create_dir_all(glb_path.parent().unwrap()).unwrap();
            let glb = generate_quantized_glb(decimated).unwrap();
            let writer = File::create(glb_path).unwrap();
            glb.to_writer_with_alignment(writer, 8).unwrap();

            tile_content
        })
        .collect();

    Ok(tile_contents)
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

    let chunk_size = 1_000_000;
    let mut start_idx = 0;
    let total_points = transformed.points.len();
    let tmp_dir_path = tempdir().unwrap();

    log::info!("start tiling at max_zoom ({})...", max_zoom);
    let start_local = std::time::Instant::now();

    // calc tile coords for max_zoom
    while start_idx < total_points {
        let end_idx = (start_idx + chunk_size).min(total_points);
        let chunk = &transformed.points[start_idx..end_idx];
        start_idx = end_idx;

        // calc tile coords
        let tile_pairs: Vec<((u8, u32, u32), Point)> = chunk
            .par_iter()
            .map(|p| {
                let tile_coords = tiling::scheme::zxy_from_lng_lat(max_zoom, p.x, p.y);
                (tile_coords, p.clone())
            })
            .collect();

        // grouping by tile
        let mut tile_map: HashMap<(u8, u32, u32), Vec<Point>> = HashMap::new();
        for (tile, pt) in tile_pairs {
            tile_map.entry(tile).or_default().push(pt);
        }

        // export to tile files
        for (tile, pts) in tile_map {
            write_points_to_tile(tmp_dir_path.path(), tile, &pts).unwrap();
        }
    }

    log::info!("Finish tiling at max_zoom in {:?}", start_local.elapsed());

    log::info!("start zoom aggregation...");
    let start_local = std::time::Instant::now();

    // The parent tile coordinates are calculated from the file with the maximum zoom level
    for z in (min_zoom..max_zoom).rev() {
        log::info!("aggregating zoom level: {}", z);
        aggregate_zoom_level(tmp_dir_path.path(), z).unwrap();
    }
    log::info!("Finish zoom aggregation in {:?}", start_local.elapsed());

    log::info!("start exporting tiles (GLB)...");
    let start_local = std::time::Instant::now();
    let tile_contents =
        export_tiles_to_glb(tmp_dir_path.path(), &output_path, min_zoom, max_zoom).unwrap();
    log::info!("Finish exporting tiles in {:?}", start_local.elapsed());

    let mut tree = TileTree::default();
    for content in tile_contents {
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

    let root_tileset_path = output_path.join("tileset.json");
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
