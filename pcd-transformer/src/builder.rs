use std::sync::Arc;

use projection_transform::{crs::EpsgCode, vshift::Jgd2011ToWgs84};

use crate::transform::{projection::ProjectionTransform, SerialTransform, Transform};

pub trait TransformBuilder {
    fn build(&self) -> Box<dyn Transform>;
}

pub struct PointCloudTransformBuilder {
    output_epsg: EpsgCode,
    jgd2wgs: Arc<Jgd2011ToWgs84>,
}

impl TransformBuilder for PointCloudTransformBuilder {
    fn build(&self) -> Box<dyn Transform> {
        let mut transformers = SerialTransform::default();

        transformers.push(Box::new(ProjectionTransform::new(
            self.jgd2wgs.clone(),
            self.output_epsg,
        )));

        Box::new(transformers)
    }
}

impl PointCloudTransformBuilder {
    pub fn new(output_epsg: EpsgCode) -> Self {
        Self {
            output_epsg,
            jgd2wgs: Jgd2011ToWgs84::default().into(),
        }
    }
}
