use super::{PointCloud, Transform};

/// A sample TilingTransform that splits the point cloud into two halves.
pub struct TilingTransform;

impl TilingTransform {
    pub fn new() -> Self {
        Self
    }
}

impl Default for TilingTransform {
    fn default() -> Self {
        Self
    }
}

impl Transform for TilingTransform {
    fn transform(&self, point_cloud: PointCloud) -> Vec<PointCloud> {
        vec![point_cloud]
    }
}
