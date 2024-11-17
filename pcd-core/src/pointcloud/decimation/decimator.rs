use std::collections::HashMap;

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
        let mut cells: HashMap<(i64, i64, i64), Vec<&Point>> = HashMap::new();

        for point in points {
            let index = self.get_voxel_index(point, voxel_size);
            cells.entry(index).or_default().push(point);
        }

        println!("  Number of cells: {}", cells.len());

        let mut decimated_points = Vec::new();
        for (index, cell_points) in cells {
            let voxel_center = self.get_voxel_center(index, voxel_size);
            let closest_point = self.select_closest_point(cell_points, voxel_center);
            decimated_points.push(closest_point.clone());
        }

        decimated_points
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

    fn select_closest_point<'a>(
        &self,
        points: Vec<&'a Point>,
        voxel_center: (f64, f64, f64),
    ) -> &'a Point {
        points
            .into_iter()
            .min_by(|a, b| {
                let dist_a = self.squared_distance(a, voxel_center);
                let dist_b = self.squared_distance(b, voxel_center);
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap()
    }

    fn squared_distance(&self, a: &Point, b: (f64, f64, f64)) -> f64 {
        (a.x - b.0).powi(2) + (a.y - b.1).powi(2) + (a.z - b.2).powi(2)
    }
}
