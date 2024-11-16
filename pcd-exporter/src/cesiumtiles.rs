use std::collections::HashMap;

use pcd_core::pointcloud::point::{BoundingVolume, Point, PointCloud};
use tinymvt::TileZXY;

use crate::tiling;

pub fn write_cesium_tiles() {
    println!("Hello, cesiumtiles!");
}

// PointCloudをタイルに分割する関数
pub fn pointcloud_to_tiles(
    pointcloud: &PointCloud,
    min_zoom: u8,
    max_zoom: u8,
) -> Vec<(TileZXY, PointCloud)> {
    // バウンディングボリュームから座標範囲を取得
    let min_x = pointcloud.metadata.bounding_volume.min[0];
    let max_x = pointcloud.metadata.bounding_volume.max[0];
    let min_y = pointcloud.metadata.bounding_volume.min[1];
    let max_y = pointcloud.metadata.bounding_volume.max[1];

    // タイルごとにポイントを格納するハッシュマップ
    let mut tile_pointclouds: HashMap<TileZXY, Vec<Point>> = HashMap::new();

    // ポイントをズームレベルごとにタイルに割り当て
    for point in &pointcloud.points {
        for zoom in min_zoom..=max_zoom {
            println!("zoom: {}", zoom);
            println!("point: {:?}", point);
            let (zoom, tile_x, tile_y) = tiling::scheme::zxy_from_lng_lat(zoom, point.x, point.y);
            let tile = (zoom, tile_x, tile_y);

            // ポイントを適切なタイルに追加
            tile_pointclouds
                .entry(tile)
                .or_insert_with(Vec::new)
                .push(point.clone());
        }
    }

    // タイルごとのPointCloudを作成
    let mut result = Vec::new();

    for ((z, x, y), points) in tile_pointclouds {
        // メタデータを更新
        let mut metadata = pointcloud.metadata.clone();
        metadata.point_count = points.len();

        // バウンディングボリュームを再計算
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
