use std::path::Path;

use pcd_core::pointcloud::point::Point;
use proj_sys_transformer::ProjTransformer;

use crate::error::ProjectionError;

/// EPSG code type alias
pub type EpsgCode = u16;

/// WGS84 Geographic 3D (EPSG:4979)
pub const EPSG_WGS84_GEOGRAPHIC_3D: EpsgCode = 4979;

pub enum PointTransformer {
    Identity,
    Proj(ProjTransformer),
}

impl PointTransformer {
    pub fn new(
        input_epsg: EpsgCode,
        output_epsg: EpsgCode,
        proj_data_dir: Option<&Path>,
    ) -> Result<Self, ProjectionError> {
        if input_epsg == output_epsg {
            return Ok(Self::Identity);
        }

        Ok(Self::Proj(ProjTransformer::new_epsg(
            input_epsg,
            output_epsg,
            proj_data_dir,
        )?))
    }

    pub fn transform_points_in_place(
        &mut self,
        points: &mut [Point],
    ) -> Result<(), ProjectionError> {
        match self {
            Self::Identity => Ok(()),
            Self::Proj(t) => t
                .transform_points_in_place(points)
                .map_err(ProjectionError::from),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pcd_core::pointcloud::point::{Color, PointAttributes};

    const EPSG_JGD2011_GEOGRAPHIC_3D: EpsgCode = 6697;

    fn make_point(x: f64, y: f64, z: f64) -> Point {
        Point {
            x,
            y,
            z,
            color: Color::default(),
            attributes: PointAttributes {
                intensity: None,
                return_number: None,
                classification: None,
                scanner_channel: None,
                scan_angle: None,
                user_data: None,
                point_source_id: None,
                gps_time: None,
            },
        }
    }

    #[test]
    fn identity_transform() {
        let mut transformer =
            PointTransformer::new(EPSG_WGS84_GEOGRAPHIC_3D, EPSG_WGS84_GEOGRAPHIC_3D, None)
                .unwrap();
        let mut points = vec![make_point(1.0, 2.0, 3.0)];
        transformer.transform_points_in_place(&mut points).unwrap();
        assert_eq!(points[0].x, 1.0);
        assert_eq!(points[0].y, 2.0);
        assert_eq!(points[0].z, 3.0);
    }

    #[test]
    fn jgd2011_geographic_to_jgd2011_geographic_is_noop() {
        let mut transformer =
            PointTransformer::new(EPSG_JGD2011_GEOGRAPHIC_3D, EPSG_JGD2011_GEOGRAPHIC_3D, None)
                .unwrap();
        let mut points = vec![make_point(140.0, 36.0, 10.0)];
        transformer.transform_points_in_place(&mut points).unwrap();
        assert_eq!(points[0].x, 140.0);
        assert_eq!(points[0].y, 36.0);
        assert_eq!(points[0].z, 10.0);
    }
}
