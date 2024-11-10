use pcd_core::pointcloud::point::{Point, PointCloud};

use super::Transform;

/// A sample ProjectionTransform that applies a simple projection to the point cloud.
pub struct ProjectionTransform;

impl ProjectionTransform {
    pub fn new() -> Self {
        Self
    }
}

impl Default for ProjectionTransform {
    fn default() -> Self {
        Self
    }
}

impl Transform for ProjectionTransform {
    fn transform(&self, point_cloud: PointCloud) -> Vec<PointCloud> {
        vec![point_cloud]
    }
}
