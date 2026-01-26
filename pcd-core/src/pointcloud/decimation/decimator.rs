use std::collections::{hash_map::Entry, HashMap};

use crate::pointcloud::point::Point;

pub trait PointCloudDecimator {
    fn decimate(&self, points: &[Point]) -> Vec<Point>;
}

pub struct VoxelDecimator {
    pub voxel_size: f64,
}

impl PointCloudDecimator for VoxelDecimator {
    fn decimate(&self, points: &[Point]) -> Vec<Point> {
        let voxel_size = self.voxel_size;
        let mut best_points: HashMap<(i64, i64, i64), (f64, &Point)> = HashMap::new();

        for point in points {
            let index = self.get_voxel_index(point, voxel_size);
            let voxel_center = self.get_voxel_center(index, voxel_size);
            let dist = self.squared_distance(point, voxel_center);

            match best_points.entry(index) {
                Entry::Vacant(e) => {
                    e.insert((dist, point));
                }
                Entry::Occupied(mut e) => {
                    let (best_dist, _) = *e.get();
                    if dist < best_dist {
                        e.insert((dist, point));
                    }
                }
            }
        }

        best_points
            .into_values()
            .map(|(_, point)| point.clone())
            .collect()
    }
}

impl VoxelDecimator {
    fn get_voxel_index(&self, point: &Point, voxel_size: f64) -> (i64, i64, i64) {
        let x_idx = (point.x / voxel_size).floor() as i64;
        let y_idx = (point.y / voxel_size).floor() as i64;
        let z_idx = (point.z / voxel_size).floor() as i64;
        (x_idx, y_idx, z_idx)
    }

    fn get_voxel_center(&self, index: (i64, i64, i64), voxel_size: f64) -> (f64, f64, f64) {
        let (x_idx, y_idx, z_idx) = index;
        (
            (x_idx as f64 + 0.5) * voxel_size,
            (y_idx as f64 + 0.5) * voxel_size,
            (z_idx as f64 + 0.5) * voxel_size,
        )
    }

    fn squared_distance(&self, a: &Point, b: (f64, f64, f64)) -> f64 {
        (a.x - b.0).powi(2) + (a.y - b.1).powi(2) + (a.z - b.2).powi(2)
    }
}
