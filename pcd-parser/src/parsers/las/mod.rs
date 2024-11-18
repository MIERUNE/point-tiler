use std::sync::mpsc::channel;
use std::thread;
use std::{error::Error, path::PathBuf};

use las::Reader;

use pcd_core::pointcloud::point::{Color, Point, PointAttributes, PointCloud};
use projection_transform::crs::EpsgCode;

use super::{Parser, ParserProvider};

pub struct LasParserProvider {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl ParserProvider for LasParserProvider {
    fn get_parser(&self) -> Box<dyn Parser> {
        Box::new(LasParser {
            filenames: self.filenames.clone(),
            epsg: self.epsg,
        })
    }
}

pub struct LasParser {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl Parser for LasParser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>> {
        let mut points = Vec::new();

        let (tx, rx) = channel();

        let handles: Vec<_> = self
            .filenames
            .iter()
            .cloned()
            .map(|filename| {
                let tx = tx.clone();
                thread::spawn(move || {
                    let mut reader = Reader::from_path(filename).unwrap();
                    for las_point in reader.points() {
                        let las_point = las_point.unwrap();

                        let color = las_point.color.map(|c| Color {
                            r: c.red,
                            g: c.green,
                            b: c.blue,
                        });

                        let attributes = PointAttributes {
                            intensity: Some(las_point.intensity),
                            return_number: Some(las_point.return_number),
                            classification: Some(format!("{:?}", las_point.classification)),
                            scanner_channel: Some(las_point.user_data),
                            scan_angle: Some(las_point.scan_angle),
                            user_data: Some(las_point.user_data),
                            point_source_id: Some(las_point.point_source_id),
                            gps_time: Some(las_point.gps_time.unwrap_or(0.0)),
                        };

                        let point = Point {
                            x: las_point.x,
                            y: las_point.y,
                            z: las_point.z,
                            color: color.unwrap_or(Color {
                                r: 65535,
                                g: 65535,
                                b: 65535,
                            }),
                            attributes,
                        };

                        tx.send(point).unwrap();
                    }
                })
            })
            .collect();

        for point in rx {
            points.push(point);
        }

        for handle in handles {
            handle.join().unwrap();
        }

        let point_cloud = PointCloud::new(points, self.epsg);

        Ok(point_cloud)
    }
}
