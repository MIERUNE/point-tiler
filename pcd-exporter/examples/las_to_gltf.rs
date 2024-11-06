use std::{io::Write as _, path::PathBuf};

use pcd_exporter::gltf::write_glb;
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};

fn main() {
    let provider = LasParserProvider {
        filenames: vec![PathBuf::from("pcd-exporter/examples/data/sample.las")],
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

    let glb_path = "pcd-exporter/examples/data/output/sample.glb";
    std::fs::create_dir_all(std::path::Path::new(glb_path).parent().unwrap()).unwrap();

    let writer = std::fs::File::create(glb_path).unwrap();
    let (gltf, _) = write_glb(writer, &point_cloud.unwrap()).unwrap();

    let gltf_path = "pcd-exporter/examples/data/output/sample.gltf";
    let mut gltf_writer = std::fs::File::create(gltf_path).unwrap();
    gltf_writer
        .write_all(serde_json::to_string_pretty(&gltf).unwrap().as_bytes())
        .unwrap();
}
