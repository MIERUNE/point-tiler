use std::{collections::HashMap, sync::mpsc, thread};

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
    let num_threads = 8;
    let chunk_size = (pointcloud.points.len() + num_threads - 1) / num_threads;

    let (tx, rx) = mpsc::sync_channel(2000);

    let mut handles = Vec::new();

    for chunk in pointcloud.points.chunks(chunk_size) {
        let tx = tx.clone();
        let chunk = chunk.to_vec();

        let handle = thread::spawn(move || {
            let mut local_map = HashMap::<TileZXY, Vec<Point>>::new();
            for point in chunk {
                for z in min_zoom..=max_zoom {
                    let (zoom, tile_x, tile_y) =
                        tiling::scheme::zxy_from_lng_lat(z, point.x, point.y);
                    let tile_coords = (zoom, tile_x, tile_y);
                    local_map
                        .entry(tile_coords)
                        .or_default()
                        .push(point.clone());
                }
            }
            tx.send(local_map).unwrap();
        });
        handles.push(handle);
    }

    drop(tx);

    let mut tile_point_clouds: HashMap<TileZXY, Vec<Point>> = HashMap::new();
    for received_map in rx {
        for (key, points) in received_map {
            tile_point_clouds.entry(key).or_default().extend(points);
        }
    }

    for handle in handles {
        handle.join().unwrap();
    }

    let epsg = pointcloud.metadata.epsg;
    let mut result = Vec::new();

    for ((z, x, y), points) in tile_point_clouds {
        let tile_pointcloud = PointCloud::new(points, epsg);
        result.push(((z, x, y), tile_pointcloud));
    }

    result
}
