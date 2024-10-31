use std::{collections::HashMap, error::Error, path::PathBuf};

use las::Reader;

use crate::pointcloud::point::{Metadata, Point, PointAttributes, PointCloud};

use super::{Parser, ParserProvider};

pub struct LasParserProvider {
    pub filenames: Vec<PathBuf>,
}

impl ParserProvider for LasParserProvider {
    fn get_parser(&self) -> Box<dyn Parser> {
        Box::new(LasParser {
            filenames: self.filenames.clone(),
        })
    }
}

pub struct LasParser {
    pub filenames: Vec<PathBuf>,
}

impl Parser for LasParser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>> {
        let start = std::time::Instant::now();
        let mut reader = Reader::from_path(&self.filenames[0]).unwrap();
        println!("Read LAS time: {:?}", start.elapsed());

        let mut points = Vec::new();

        let start = std::time::Instant::now();
        for las_point in reader.points() {
            let las_point = las_point.unwrap();

            let attributes = PointAttributes {
                intensity: Some(las_point.intensity),
                return_number: Some(las_point.return_number),
                classification: Some(format!("{:?}", las_point.classification)),
                scanner_channel: Some(las_point.user_data),
                scan_angle: Some(las_point.scan_angle),
                user_data: Some(las_point.user_data),
                point_source_id: Some(las_point.point_source_id),
                gps_time: Some(las_point.gps_time.unwrap_or(0.0)),
                r: Some(las_point.color.map(|c| c.red).unwrap_or(0)),
                g: Some(las_point.color.map(|c| c.green).unwrap_or(0)),
                b: Some(las_point.color.map(|c| c.blue).unwrap_or(0)),
            };

            let point = Point {
                x: las_point.x,
                y: las_point.y,
                z: las_point.z,
                attributes,
            };

            points.push(point);
        }
        println!("Build PointCloud time: {:?}", start.elapsed());

        // todo: メタデータ取り込み部分を作る
        let metadata = Metadata {
            coordinate_system_wkt: "PROJCS[\"JGD2011 / Japan Plane Rectangular CS VII\",...]"
                .to_string(),
            scale: [0.001, 0.001, 0.001],
            offset: [0.0, 0.0, 0.0],
            max_xyz: [10000.0, 20000.0, 500.0],
            min_xyz: [0.0, 0.0, 0.0],
            other: HashMap::new(),
        };

        Ok(PointCloud { points, metadata })
    }
}
