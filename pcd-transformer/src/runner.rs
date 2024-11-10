use pcd_core::pointcloud::point::PointCloud;

use crate::transform::Transform;

pub trait Transformer {
    fn execute(&self, point_cloud: PointCloud) -> Vec<PointCloud>;
}

pub struct PointCloudTransformer {
    transform: Box<dyn Transform>,
}

impl PointCloudTransformer {
    pub fn new(transform: Box<dyn Transform>) -> Self {
        Self { transform }
    }
}

impl Transformer for PointCloudTransformer {
    fn execute(&self, point_cloud: PointCloud) -> Vec<PointCloud> {
        self.transform.transform(point_cloud)
    }
}
