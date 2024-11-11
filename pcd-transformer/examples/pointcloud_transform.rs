use std::path::PathBuf;

use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};
use pcd_transformer::{
    builder::PointCloudTransformBuilder,
    runner::{PointCloudTransformer, Transformer},
};

fn main() {
    let las_parser_provider = LasParserProvider {
        filenames: vec![PathBuf::from(
            "pcd-transformer/examples/data/sample.las".to_string(),
        )],
        epsg: 10170,
    };
    let provider = las_parser_provider;
    let parser = provider.get_parser();
    let point_cloud = parser.parse().unwrap();

    let transform_builder = PointCloudTransformBuilder::new(4979);
    let transformer = PointCloudTransformer::new(Box::new(transform_builder));

    let transformed = transformer.execute(point_cloud);

    for (i, pc) in transformed.iter().enumerate() {
        println!("PointCloud {}:", i);
        println!("  Point(x: {}, y: {}, z: {})", pc.0, pc.1, pc.2);
    }
}
