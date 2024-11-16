use std::collections::HashMap;

use pcd_core::pointcloud::point::{Point, PointCloud};
use tinymvt::TileZXY;

use crate::tiling::{self, TileContent};

pub type TiledPointCloud = (TileZXY, PointCloud);

pub fn make_tile_contents(tiled_pointcloud: &TiledPointCloud) -> TileContent {
    let (tile, pointcloud) = tiled_pointcloud;

    let (tile_zoom, tile_x, tile_y) = tile;

    let min_lng = pointcloud.metadata.bounding_volume.min[0];
    let max_lng = pointcloud.metadata.bounding_volume.max[0];
    let min_lat = pointcloud.metadata.bounding_volume.min[1];
    let max_lat = pointcloud.metadata.bounding_volume.max[1];
    let min_height = pointcloud.metadata.bounding_volume.min[2];
    let max_height = pointcloud.metadata.bounding_volume.max[2];

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
        let epsg = pointcloud.metadata.epsg;
        let tile_pointcloud = PointCloud::new(points, epsg);
        result.push(((z, x, y), tile_pointcloud));
    }

    result
}
