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

        // Sort by Morton code for spatial locality (improves meshopt compression)
        let mut indexed: Vec<_> = best_points
            .into_iter()
            .map(|((x, y, z), (_, point))| (morton_encode_3d(x, y, z), point.clone()))
            .collect();
        indexed.sort_unstable_by_key(|(morton, _)| *morton);
        indexed.into_iter().map(|(_, point)| point).collect()
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

/// 3D Morton encoding (bit interleaving) for spatial sorting.
/// Uses 21 bits per coordinate (fits in u64).
fn morton_encode_3d(x: i64, y: i64, z: i64) -> u64 {
    fn expand_bits(mut v: u64) -> u64 {
        v &= 0x1fffff; // 21 bits
        v = (v | (v << 32)) & 0x1f00000000ffff;
        v = (v | (v << 16)) & 0x1f0000ff0000ff;
        v = (v | (v << 8)) & 0x100f00f00f00f00f;
        v = (v | (v << 4)) & 0x10c30c30c30c30c3;
        v = (v | (v << 2)) & 0x1249249249249249;
        v
    }
    // Shift to unsigned by adding offset (to handle negative indices)
    let ux = (x.wrapping_add(1 << 20)) as u64;
    let uy = (y.wrapping_add(1 << 20)) as u64;
    let uz = (z.wrapping_add(1 << 20)) as u64;
    expand_bits(ux) | (expand_bits(uy) << 1) | (expand_bits(uz) << 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_morton_encode_3d_origin() {
        let code = morton_encode_3d(0, 0, 0);
        assert!(code > 0); // offset ensures non-zero
    }

    #[test]
    fn test_morton_encode_3d_spatial_locality() {
        // Nearby points should have closer Morton codes than distant points
        let near_a = morton_encode_3d(10, 10, 10);
        let near_b = morton_encode_3d(11, 10, 10);
        let far = morton_encode_3d(1000, 1000, 1000);

        let diff_near = near_a.abs_diff(near_b);
        let diff_far = near_a.abs_diff(far);
        assert!(diff_near < diff_far);
    }

    #[test]
    fn test_morton_encode_3d_negative_indices() {
        // Should handle negative indices without panicking
        let _ = morton_encode_3d(-1, -1, -1);
        let _ = morton_encode_3d(-100, 50, -200);
    }
}
