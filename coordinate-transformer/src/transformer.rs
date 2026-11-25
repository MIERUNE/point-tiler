use std::path::Path;

use pcd_core::pointcloud::point::Point;
use proj::{Proj, ProjBuilder};

use crate::error::ProjError;

/// EPSG code type alias
pub type EpsgCode = u16;

/// WGS84 Geographic 3D (EPSG:4979)
pub const EPSG_WGS84_GEOGRAPHIC_3D: EpsgCode = 4979;

/// WGS84 Geocentric / Earth-Centered Earth-Fixed (EPSG:4978)
pub const EPSG_WGS84_GEOCENTRIC: EpsgCode = 4978;

/// Transforms points between two CRSs. `proj::Proj` is neither `Send` nor `Sync`, so create
/// one `PointTransformer` per thread.
#[derive(Debug)]
pub struct PointTransformer {
    proj: Proj,
}

impl PointTransformer {
    pub fn new(
        input_epsg: EpsgCode,
        output_epsg: EpsgCode,
        proj_data_dir: Option<&Path>,
    ) -> Result<Self, ProjError> {
        let source = format!("EPSG:{input_epsg}");
        let target = format!("EPSG:{output_epsg}");
        Self::new_from_crs(&source, &target, proj_data_dir)
    }

    fn new_from_crs(
        source: &str,
        target: &str,
        proj_data_dir: Option<&Path>,
    ) -> Result<Self, ProjError> {
        let mut builder = ProjBuilder::new();
        if let Some(dir) = proj_data_dir {
            builder
                .set_search_paths(dir)
                .map_err(|e| ProjError::new("set_search_paths", e))?;
        }
        // `proj_known_crs` normalizes the axis order to lon/lat (easting/northing), e.g.
        // EPSG:4326 is lat,lon by definition.
        let proj = builder
            .proj_known_crs(source, target, None)
            .map_err(|e| ProjError::new("proj_known_crs", e))?;

        Ok(Self { proj })
    }

    pub fn transform_points_in_place(&mut self, points: &mut [Point]) -> Result<(), ProjError> {
        for p in points {
            (p.x, p.y, p.z) = self
                .proj
                .convert((p.x, p.y, p.z))
                .map_err(|e| ProjError::new("convert", e))?;
        }
        Ok(())
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
