use pcd_core::pointcloud::point::PointCloud;

pub mod projection;

pub trait Transform {
    fn transform(&self, point_cloud: PointCloud) -> Vec<PointCloud>;
}

pub struct CompositeTransform {
    transforms: Vec<Box<dyn Transform>>,
}

impl CompositeTransform {
    pub fn new(transforms: Vec<Box<dyn Transform>>) -> Self {
        Self { transforms }
    }
}

impl Transform for CompositeTransform {
    fn transform(&self, point_cloud: PointCloud) -> Vec<PointCloud> {
        let mut intermediate = vec![point_cloud];

        for transform in &self.transforms {
            let mut next_stage = Vec::new();
            for pc in intermediate {
                let transformed = transform.transform(pc);
                next_stage.extend(transformed);
            }
            intermediate = next_stage;
        }

        intermediate
    }
}
