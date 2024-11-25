use std::collections::HashMap;

use pcd_core::pointcloud::point::{Point, PointCloud};
use tinymvt::TileZXY;

use crate::tiling::{self, TileContent};

pub fn make_tile_content(tile_coord: &TileZXY, point_cloud: &PointCloud) -> TileContent {
    let (tile_zoom, tile_x, tile_y) = tile_coord;

    let min_lng = point_cloud.metadata.bounding_volume.min[0];
    let max_lng = point_cloud.metadata.bounding_volume.max[0];
    let min_lat = point_cloud.metadata.bounding_volume.min[1];
    let max_lat = point_cloud.metadata.bounding_volume.max[1];
    let min_height = point_cloud.metadata.bounding_volume.min[2];
    let max_height = point_cloud.metadata.bounding_volume.max[2];

    let content_path = { format!("{tile_zoom}/{tile_x}/{tile_y}.glb") };

    TileContent {
        zxy: (*tile_zoom, *tile_x, *tile_y),
        content_path,
        min_lng,
        max_lng,
        min_lat,
        max_lat,
        min_height,
        max_height,
    }
}

pub fn pointcloud_to_tiles(
    pointcloud: &PointCloud,
    min_zoom: u8,
    max_zoom: u8,
) -> Vec<(TileZXY, PointCloud)> {
    let mut tile_point_clouds: HashMap<TileZXY, Vec<Point>> = HashMap::new();

    for point in &pointcloud.points {
        for z in min_zoom..=max_zoom {
            let (zoom, tile_x, tile_y) = tiling::scheme::zxy_from_lng_lat(z, point.x, point.y);
            let tile_coords = (zoom, tile_x, tile_y);

            tile_point_clouds
                .entry(tile_coords)
                .or_default()
                .push(point.clone());
        }
    }

    let mut result = Vec::new();

    for ((z, x, y), points) in tile_point_clouds {
        let epsg = pointcloud.metadata.epsg;
        let tile_pointcloud = PointCloud::new(points, epsg);
        result.push(((z, x, y), tile_pointcloud));
    }

    result
}
