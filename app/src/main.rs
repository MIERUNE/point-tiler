use std::io::Write;
use std::{
    fs,
    path::{Path, PathBuf},
};

use chrono::Local;
use env_logger::Builder;
use log::LevelFilter;

use pcd_core::pointcloud::{
    decimation::decimator::{PointCloudDecimator, VoxelDecimator},
    point::{Point, PointCloud},
};
use pcd_exporter::{
    cesiumtiles::{make_tile_content, pointcloud_to_tiles},
    gltf::generate_quantized_glb,
    tiling::{geometric_error, TileContent, TileTree},
};
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
    let input_files = args.input.iter().map(PathBuf::from).collect::<Vec<_>>();
    let output_path = PathBuf::from(args.output);
    std::fs::create_dir_all(&output_path).unwrap();

    log::info!("start parsing...");
    let las_parser_provider = LasParserProvider {
        filenames: input_files,
        epsg: args.epsg,
    };
    let output_epsg = 4979;
    let provider = las_parser_provider;
    let parser = provider.get_parser();
    let point_cloud = parser.parse().unwrap();
    log::info!("finish parsing");

    log::info!("start transforming...");
    let transform_builder = PointCloudTransformBuilder::new(output_epsg);
    let transformer = PointCloudTransformer::new(Box::new(transform_builder));

    let transformed = transformer.execute(point_cloud.clone());
    log::info!("Finish transforming");

    let min_zoom = args.min;
    let max_zoom = args.max;
    log::info!("start tiling...");
    let tiled_pointcloud = pointcloud_to_tiles(&transformed, min_zoom, max_zoom);
    log::info!("Finish tiling");

    log::info!("start exporting tiles...");
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
