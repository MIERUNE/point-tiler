use std::sync::Arc;

use pcd_core::pointcloud::point::PointCloud;
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
    fn transform(&self, point_cloud: PointCloud) -> Vec<PointCloud> {
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
                self.transform_from_jgd2011(&point_cloud, Some(input_epsg));
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

    fn transform_from_jgd2011(&mut self, point_cloud: &PointCloud, rectangular: Option<EpsgCode>) {
        match self.output_epsg {
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
                // To Japan Plane Rectangular CS + JGD2011 (vertical) height
                let proj = self.jpr_zone_proj.as_ref().unwrap();
                let mut geom_store = entity.geometry_store.write().unwrap();
                geom_store.vertices.iter_mut().for_each(|v| {
                    let (lng, lat) = (v[1], v[0]);
                    if let Some(input_epsg) = rectangular {
                        (v[0], v[1], v[2]) =
                            Self::rectangular_to_lnglat(v[0], v[1], v[2], input_epsg);
                    };
                    // Change x and y; keep the height
                    // TODO: error handling
                    (v[0], v[1], _) = proj.project_forward(lng, lat, 0.).unwrap();
                });
                geom_store.epsg = self.output_epsg;
            }
            EPSG_JGD2011_JPRECT_I
            | EPSG_JGD2011_JPRECT_II
            | EPSG_JGD2011_JPRECT_III
            | EPSG_JGD2011_JPRECT_IV
            | EPSG_JGD2011_JPRECT_V
            | EPSG_JGD2011_JPRECT_VI
            | EPSG_JGD2011_JPRECT_VII
            | EPSG_JGD2011_JPRECT_VIII
            | EPSG_JGD2011_JPRECT_IX
            | EPSG_JGD2011_JPRECT_X
            | EPSG_JGD2011_JPRECT_XI
            | EPSG_JGD2011_JPRECT_XII
            | EPSG_JGD2011_JPRECT_XIII
            | EPSG_JGD2011_JPRECT_XIV
            | EPSG_JGD2011_JPRECT_XV
            | EPSG_JGD2011_JPRECT_XVI
            | EPSG_JGD2011_JPRECT_XVII
            | EPSG_JGD2011_JPRECT_XVIII
            | EPSG_JGD2011_JPRECT_XIX => {
                // To Japan Plane Rectangular CS
                let proj = self.jpr_zone_proj.as_ref().unwrap();
                let mut geom_store = entity.geometry_store.write().unwrap();
                geom_store.vertices.iter_mut().for_each(|v| {
                    let (lng, lat) = (v[1], v[0]);
                    if let Some(input_epsg) = rectangular {
                        (v[0], v[1], v[2]) =
                            Self::rectangular_to_lnglat(v[0], v[1], v[2], input_epsg);
                    };
                    // Change x and y; keep the height
                    // TODO: error handling
                    (v[0], v[1], _) = proj.project_forward(lng, lat, 0.).unwrap();
                });
                geom_store.epsg = self.output_epsg;
            }
            _ => {
                panic!("Unsupported output CRS: {}", self.output_epsg);
            }
        };
    }
}
