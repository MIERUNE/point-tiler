use std::path::PathBuf;

use pcd_core::pointcloud::point::{Point, PointCloud};
use pcd_exporter::{cesiumtiles::pointcloud_to_tiles, gltf::generate_quantized_glb};
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};
use pcd_transformer::{
    builder::PointCloudTransformBuilder,
    runner::{PointCloudTransformer, Transformer},
};
use projection_transform::cartesian::geodetic_to_geocentric;

fn main() {
    let las_parser_provider = LasParserProvider {
        filenames: vec![PathBuf::from(
            "pcd-transformer/examples/data/sample.las".to_string(),
        )],
        epsg: 6677,
    };
    let output_epsg = 4979;
    let provider = las_parser_provider;
    let parser = provider.get_parser();
    let point_cloud = parser.parse().unwrap();

    let transform_builder = PointCloudTransformBuilder::new(output_epsg);
    let transformer = PointCloudTransformer::new(Box::new(transform_builder));

    let transformed = transformer.execute(point_cloud);

    println!(
        "Number of points: {num_points}",
        num_points = transformed.points.len()
    );

    let mut points = vec![];
    let ellipsoid = projection_transform::ellipsoid::wgs84();
    for point in transformed.iter() {
        let (lng, lat, height) = (point.0, point.1, point.2);
        let (x, y, z) = geodetic_to_geocentric(&ellipsoid, lng, lat, height);
        points.push(Point {
            x,
            y,
            z,
            color: point.3.color.clone(),
            attributes: point.3.attributes.clone(),
        });
    }
    let transformed = PointCloud::new(points, output_epsg);

    println!("First point: {:?}", transformed.points[0]);
    println!("PointCloud metadata: {:?}", transformed.metadata);

    let glb_path = "pcd-transformer/examples/data/output/sample.glb";
    std::fs::create_dir_all(std::path::Path::new(glb_path).parent().unwrap()).unwrap();

    let glb = generate_quantized_glb(transformed).unwrap();

    let writer = std::fs::File::create(glb_path).unwrap();
    let _ = glb.to_writer_with_alignment(writer, 8);
}
