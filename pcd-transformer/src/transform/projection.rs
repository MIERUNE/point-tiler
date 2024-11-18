use std::{
    sync::{mpsc, Arc},
    thread,
};

use pcd_core::pointcloud::point::{Point, PointCloud};
use projection_transform::{crs::*, jprect::JPRZone, vshift::Jgd2011ToWgs84};

use super::Transform;

pub struct ProjectionTransform {
    jgd2wgs: Arc<Jgd2011ToWgs84>,
    output_epsg: EpsgCode,
}

impl Transform for ProjectionTransform {
    fn transform(&self, point_cloud: PointCloud) -> PointCloud {
        let input_epsg = point_cloud.metadata.epsg;

        match input_epsg {
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
            | EPSG_JGD2011_JPRECT_XIX
            | EPSG_JGD2011_JPRECT_I_JGD2011_HEIGHT
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
                self.transform_from_jgd2011(point_cloud, Some(input_epsg))
            }
            _ => {
                panic!("Unsupported input CRS: {}", input_epsg);
            }
        }
    }
}

impl ProjectionTransform {
    pub fn new(jgd2wgs: Arc<Jgd2011ToWgs84>, output_epsg: EpsgCode) -> Self {
        Self {
            jgd2wgs,
            output_epsg,
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
                let (tx, rx) = mpsc::sync_channel(2000);
                let num_threads = 8;
                let chunk_size = (point_cloud.points.len() + num_threads - 1) / num_threads;

                let mut handles = vec![];

                let jgd2wgs_arc = Arc::new(self.jgd2wgs.clone());

                for chunk in point_cloud.points.chunks(chunk_size) {
                    let tx = tx.clone();
                    let jgd2wgs_arc = Arc::clone(&jgd2wgs_arc);

                    let chunk = chunk.to_vec();

                    let handle = thread::spawn(move || {
                        for point in chunk {
                            let x = point.x;
                            let y = point.y;
                            let z = point.z;

                            let (lng, lat, height) = if let Some(input_epsg) = rectangular {
                                Self::rectangular_to_lnglat(x, y, z, input_epsg)
                            } else {
                                (x, y, z)
                            };

                            let (lng, lat, height) = jgd2wgs_arc.convert(lng, lat, height);

                            let new_point = Point {
                                x: lng,
                                y: lat,
                                z: height,
                                color: point.color.clone(),
                                attributes: point.attributes.clone(),
                            };

                            tx.send(new_point).unwrap();
                        }
                    });

                    handles.push(handle);
                }

                drop(tx);

                for processed_point in rx {
                    points.push(processed_point);
                }

                for handle in handles {
                    handle.join().unwrap();
                }
            }
            _ => {
                panic!("Unsupported output CRS: {}", self.output_epsg);
            }
        }

        PointCloud::new(points, self.output_epsg)
    }
}
