use std::sync::Arc;

use pcd_core::pointcloud::point::{Point, PointCloud};
use projection_transform::{
    crs::*, etmerc::ExtendedTransverseMercatorProjection, jprect::JPRZone, vshift::Jgd2011ToWgs84,
};

use super::Transform;

pub struct ProjectionTransform {
    jgd2wgs: Arc<Jgd2011ToWgs84>,
    output_epsg: EpsgCode,
    jpr_zone_proj: Option<ExtendedTransverseMercatorProjection>,
}

impl Transform for ProjectionTransform {
    fn transform(&self, point_cloud: PointCloud) -> PointCloud {
        let input_epsg = point_cloud.metadata.epsg;

        match input_epsg {
            EPSG_JGD2011_JPRECT_I_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_II_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_III_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_IV_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_V_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_VI_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_VII_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_VIII_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_IX_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_X_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_XI_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_XII_JGD2011_HEIGHT
            | EPSG_JGD2011_JPRECT_XIII_JGD2011_HEIGHT => {
                self.transform_from_jgd2011(point_cloud, None)
            }
            _ => {
                panic!("Unsupported input CRS: {}", input_epsg);
            }
        }
    }
}

impl ProjectionTransform {
    pub fn new(jgd2wgs: Arc<Jgd2011ToWgs84>, output_epsg: EpsgCode) -> Self {
        // For Japan Plane Rectangular CS
        let jpr_zone_proj = JPRZone::from_epsg(output_epsg).map(|zone| zone.projection());

        Self {
            jgd2wgs,
            output_epsg,
            jpr_zone_proj,
        }
    }

    fn rectangular_to_lnglat(x: f64, y: f64, height: f64, input_epsg: EpsgCode) -> (f64, f64, f64) {
        let zone = JPRZone::from_epsg(input_epsg).unwrap();
        let proj = zone.projection();
        let (lng, lat, height) = proj.project_inverse(x, y, height).unwrap();
        (lng, lat, height)
    }

    fn transform_from_jgd2011(
        &self,
        point_cloud: PointCloud,
        rectangular: Option<EpsgCode>,
    ) -> PointCloud {
        let mut points = vec![];
        match self.output_epsg {
            EPSG_WGS84_GEOGRAPHIC_3D => {
                for (x, y, z, point) in point_cloud.iter() {
                    // Swap x and y (lat, lng -> lng, lat)
                    let (lng, lat, height) = (y, x, z);
                    let (lng, lat, height) = if let Some(input_epsg) = rectangular {
                        Self::rectangular_to_lnglat(lng, lat, height, input_epsg)
                    } else {
                        (lng, lat, height)
                    };
                    // JGD2011 to WGS 84 (elevation to ellipsoidal height)
                    let (lng, lat, height) = self.jgd2wgs.convert(lng, lat, height);
                    points.push(Point {
                        x: lng,
                        y: lat,
                        z: height,
                        color: point.color.clone(),
                        attributes: point.attributes.clone(),
                    });
                }
            }
            _ => {
                panic!("Unsupported output CRS: {}", self.output_epsg);
            }
        };
        PointCloud::new(points, self.output_epsg)
    }
}
