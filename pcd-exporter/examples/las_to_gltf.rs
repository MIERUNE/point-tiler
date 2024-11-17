use std::{
    fs,
    path::{Path, PathBuf},
};

use pcd_core::pointcloud::point::{Point, PointCloud};
use pcd_exporter::{
    cesiumtiles::{make_tile_content, pointcloud_to_tiles},
    gltf::{generate_glb, generate_quantized_glb},
    tiling::{TileContent, TileTree},
};
use pcd_parser::parsers::{las::LasParserProvider, ParserProvider as _};
use pcd_transformer::{
    builder::PointCloudTransformBuilder, runner::PointCloudTransformer, Transformer,
};
use projection_transform::cartesian::geodetic_to_geocentric;

fn main() {
    let input_files = vec![PathBuf::from(
        "/Users/satoru/Downloads/09LD1885.las".to_string(),
        // "pcd-exporter/examples/data/sample.las".to_string(),
    )];
    let output_path = PathBuf::from(
        "/Users/satoru/Downloads/plateau/plateau-tutorial/output/3dtiles_tokyo_pointcloud",
    );
    std::fs::create_dir_all(&output_path).unwrap();

    let las_parser_provider = LasParserProvider {
        filenames: input_files,
        epsg: 6677,
    };
    let output_epsg = 4979;
    let provider = las_parser_provider;
    let parser = provider.get_parser();
    let point_cloud = parser.parse().unwrap();
    println!("first point: {:?}", point_cloud.points[0]);

    let transform_builder = PointCloudTransformBuilder::new(output_epsg);
    let transformer = PointCloudTransformer::new(Box::new(transform_builder));

    let transformed = transformer.execute(point_cloud.clone());
    print!("Transformed first point: {:?}", transformed.points[0]);

    println!(
        "Number of points: {num_points}",
        num_points = transformed.points.len()
    );

    let min_zoom = 18;
    let max_zoom = 18;
    let tiled_pointcloud = pointcloud_to_tiles(&transformed, min_zoom, max_zoom);

    let mut tile_contents: Vec<TileContent> = Default::default();
    for (tile_coords, pointcloud) in tiled_pointcloud {
        println!("Tile Coords: {:?}", tile_coords);
        println!(
            "  Number of points: {num_points}",
            num_points = pointcloud.points.len()
        );

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
        println!("transformed first point: {:?}", transformed.points[0]);
        println!("offset: {:?}", transformed.metadata.offset);

        let glb_path = format!(
            "{}/{}",
            output_path.to_string_lossy(),
            tile_content.content_path
        );
        println!("write GLB: {:?}", glb_path);
        std::fs::create_dir_all(std::path::Path::new(&glb_path).parent().unwrap()).unwrap();

        let glb = generate_quantized_glb(transformed).unwrap();

        let writer = std::fs::File::create(glb_path).unwrap();
        let _ = glb.to_writer_with_alignment(writer, 8);

        tile_contents.push(tile_content);
    }

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
    println!("write tileset.json: {:?}", root_tileset_path);
    fs::create_dir_all(root_tileset_path.parent().unwrap()).unwrap();
    fs::write(
        root_tileset_path,
        serde_json::to_string_pretty(&tileset).unwrap(),
    )
    .unwrap();
}
