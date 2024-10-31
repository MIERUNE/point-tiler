use std::path::PathBuf;

use pcd_parser::parsers::{csv::CsvParserProvider, ParserProvider as _};

fn main() {
    let provider = CsvParserProvider {
        filenames: vec![PathBuf::from("examples/data/sample.txt")],
    };
    let parser = provider.get_parser();

    let point_cloud = parser.parse();

    println!(
        "Number of points: {num_points}",
        num_points = point_cloud.as_ref().unwrap().points.len()
    );

    println!("First point: {:?}", point_cloud.as_ref().unwrap().points[0]);
}
