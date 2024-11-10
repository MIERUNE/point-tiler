use crate::transform::{CompositeTransform, Transform};

pub trait TransformBuilder {
    fn build(&self) -> Box<dyn Transform>;
}

pub struct TilingAndProjectionTransformBuilder;

impl TransformBuilder for TilingAndProjectionTransformBuilder {
    fn build(&self) -> Box<dyn Transform> {
        let tiling_transform = Box::new(crate::transform::tile::TilingTransform::new());
        let projection_transform =
            Box::new(crate::transform::projection::ProjectionTransform::new());

        let composite = CompositeTransform::new(vec![tiling_transform, projection_transform]);

        Box::new(composite)
    }
}
