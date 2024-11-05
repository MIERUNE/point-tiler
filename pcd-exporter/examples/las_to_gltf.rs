use std::path::PathBuf;

use pcd_exporter::gltf::write_gltf;
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};

fn main() {
    let provider = LasParserProvider {
        filenames: vec![PathBuf::from("pcd-parser/examples/data/sample.las")],
    };
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

    let writer = std::fs::File::create("pcd-parser/examples/data/output.glb").unwrap();
    let _ = write_gltf(writer, &point_cloud.unwrap());
}
