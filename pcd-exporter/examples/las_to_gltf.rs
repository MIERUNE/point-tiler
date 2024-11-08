use std::path::PathBuf;

use pcd_exporter::gltf::generate_glb;
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};

fn main() {
    let las_parser_provider = LasParserProvider {
        filenames: vec![PathBuf::from("pcd-exporter/examples/data/sample.las")],
    };
    let provider = las_parser_provider;
    let parser = provider.get_parser();

    let point_cloud = parser.parse();

    println!(
        "Number of points: {num_points}",
        num_points = point_cloud.as_ref().unwrap().points.len()
    );

    println!("First point: {:?}", point_cloud.as_ref().unwrap().points[0]);
    println!(
        "PointCloud metadata: {:?}",
        point_cloud.as_ref().unwrap().metadata
    );

    let glb_path = "pcd-exporter/examples/data/output/sample.glb";
    std::fs::create_dir_all(std::path::Path::new(glb_path).parent().unwrap()).unwrap();

    let glb = generate_glb(point_cloud.unwrap()).unwrap();

    let writer = std::fs::File::create(glb_path).unwrap();
    let _ = glb.to_writer_with_alignment(writer, 8);
}
