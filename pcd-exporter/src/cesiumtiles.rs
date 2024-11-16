use std::collections::HashMap;

use pcd_core::pointcloud::point::{BoundingVolume, Point, PointCloud};
use tinymvt::TileZXY;

use crate::tiling;

pub fn write_cesium_tiles() {
    println!("Hello, cesiumtiles!");
}

pub fn pointcloud_to_tiles(
    pointcloud: &PointCloud,
    min_zoom: u8,
    max_zoom: u8,
) -> Vec<(TileZXY, PointCloud)> {
    let mut tile_pointclouds: HashMap<TileZXY, Vec<Point>> = HashMap::new();

    for point in &pointcloud.points {
        for zoom in min_zoom..=max_zoom {
            println!("zoom: {}", zoom);
            println!("point: {:?}", point);
            let (zoom, tile_x, tile_y) = tiling::scheme::zxy_from_lng_lat(zoom, point.x, point.y);
            let tile = (zoom, tile_x, tile_y);

            tile_pointclouds
                .entry(tile)
                .or_insert_with(Vec::new)
                .push(point.clone());
        }
    }

    let mut result = Vec::new();

    for ((z, x, y), points) in tile_pointclouds {
        let mut metadata = pointcloud.metadata.clone();
        metadata.point_count = points.len();

        let mut bounding_volume = BoundingVolume {
            min: [f64::MAX, f64::MAX, f64::MAX],
            max: [f64::MIN, f64::MIN, f64::MIN],
        };

        for point in &points {
            bounding_volume.max[0] = bounding_volume.max[0].max(point.x);
            bounding_volume.max[1] = bounding_volume.max[1].max(point.y);
            bounding_volume.max[2] = bounding_volume.max[2].max(point.z);
            bounding_volume.min[0] = bounding_volume.min[0].min(point.x);
            bounding_volume.min[1] = bounding_volume.min[1].min(point.y);
            bounding_volume.min[2] = bounding_volume.min[2].min(point.z);
        }

        metadata.bounding_volume = bounding_volume;

        let tile_pointcloud = PointCloud { points, metadata };

        result.push(((z, x, y), tile_pointcloud));
    }

    result
}
