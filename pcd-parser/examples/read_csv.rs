use std::path::PathBuf;

use pcd_parser::parser::{csv::CsvParserProvider, ParserProvider as _};

fn main() {
    let provider = CsvParserProvider {
        filenames: vec![PathBuf::from("pcd-parser/examples/data/sample.txt")],
        epsg: 6677,
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
}
