use std::path::PathBuf;

use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};
use pcd_transformer::{
    builder::ProjectionTransformerBuilder,
    runner::{PointCloudTransformer, Transformer},
    TransformBuilder as _,
};

fn main() {
    let las_parser_provider = LasParserProvider {
        filenames: vec![PathBuf::from(
            "pcd-transformer/examples/data/sample.las".to_string(),
        )],
    };
    let provider = las_parser_provider;
    let parser = provider.get_parser();
    let point_cloud = parser.parse().unwrap();

    let builder = ProjectionTransformerBuilder;
    let transform = builder.build();
    let transformer = PointCloudTransformer::new(transform);

    let transformed = transformer.execute(point_cloud);

    for (i, pc) in transformed.iter().enumerate() {
        println!("PointCloud {}:", i);
        for point in &pc.points {
            println!("  Point(x: {}, y: {}, z: {})", point.x, point.y, point.z);
        }
    }
}
