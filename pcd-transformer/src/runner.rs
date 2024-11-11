use pcd_core::pointcloud::point::PointCloud;

use crate::TransformBuilder;

pub trait Transformer {
    fn execute(&self, point_cloud: PointCloud) -> PointCloud;
}

pub struct PointCloudTransformer {
    builder: Box<dyn TransformBuilder>,
}

impl PointCloudTransformer {
    pub fn new(builder: Box<dyn TransformBuilder>) -> Self {
        Self { builder }
    }
}

impl Transformer for PointCloudTransformer {
    fn execute(&self, point_cloud: PointCloud) -> PointCloud {
        let transform = self.builder.build();
        transform.transform(point_cloud)
    }
}
