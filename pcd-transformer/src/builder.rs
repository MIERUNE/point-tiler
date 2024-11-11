use crate::transform::{CompositeTransform, Transform};

pub trait TransformBuilder {
    fn build(&self) -> Box<dyn Transform>;
}

pub struct ProjectionTransformerBuilder;

impl TransformBuilder for ProjectionTransformerBuilder {
    fn build(&self) -> Box<dyn Transform> {
        let projection_transform =
            Box::new(crate::transform::projection::ProjectionTransform::new());

        let composite = CompositeTransform::new(vec![projection_transform]);

        Box::new(composite)
    }
}
