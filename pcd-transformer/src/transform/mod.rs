use pcd_core::pointcloud::point::PointCloud;

pub mod projection;

pub trait Transform {
    fn transform(&self, point_cloud: PointCloud) -> PointCloud;
}

#[derive(Default)]
pub struct SerialTransform {
    transforms: Vec<Box<dyn Transform>>,
}

impl Transform for SerialTransform {
    fn transform(&self, point_cloud: PointCloud) -> PointCloud {
        let mut transformed = point_cloud;
        for transform in &self.transforms {
            transformed = transform.transform(transformed);
        }
        transformed
    }
}

impl SerialTransform {
    pub fn push(&mut self, transform: Box<dyn Transform>) {
        self.transforms.push(transform);
    }
}
