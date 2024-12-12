use std::{
    io::{self},
    path::PathBuf,
};

use las::Reader;
use pcd_core::pointcloud::point::{Color, Point, PointAttributes};

use super::PointReader;

pub struct LasPointReader {
    pub files: Vec<PathBuf>,
    pub current_file_index: usize,
    pub current_reader: Option<Reader>,
}

impl LasPointReader {
    pub fn new(files: Vec<PathBuf>) -> io::Result<Self> {
        Ok(Self {
            files,
            current_file_index: 0,
            current_reader: None,
        })
    }

    fn open_next_file(&mut self) -> io::Result<()> {
        if self.current_file_index < self.files.len() {
            let file = &self.files[self.current_file_index];
            let reader = las::Reader::from_path(file).unwrap();
            self.current_reader = Some(reader);
            self.current_file_index += 1;
            Ok(())
        } else {
            self.current_reader = None;
            Ok(())
        }
    }

    fn convert_las_point(las_point: las::Point) -> Point {
        let color = las_point
            .color
            .map(|c| Color {
                r: c.red,
                g: c.green,
                b: c.blue,
            })
            .unwrap_or(Color {
                r: 65535,
                g: 65535,
                b: 65535,
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

        Point {
            x: las_point.x,
            y: las_point.y,
            z: las_point.z,
            color,
            attributes,
        }
    }
}

impl PointReader for LasPointReader {
    fn next_point(&mut self) -> io::Result<Option<Point>> {
        loop {
            if self.current_reader.is_none() {
                self.open_next_file()?;
                if self.current_reader.is_none() {
                    return Ok(None);
                }
            }

            let reader = self.current_reader.as_mut().unwrap();
            match reader.points().next() {
                Some(Ok(las_point)) => {
                    let p = Self::convert_las_point(las_point);
                    return Ok(Some(p));
                }
                Some(Err(e)) => {
                    eprintln!("Error reading LAS point: {}", e);
                    return Err(io::Error::new(io::ErrorKind::Other, e));
                }
                None => {
                    self.current_reader = None;
                }
            }
        }
    }
}

pub struct PointIterator<R: PointReader> {
    reader: R,
    chunk_size: usize,
}

impl<R: PointReader> PointIterator<R> {
    pub fn new(reader: R, chunk_size: usize) -> Self {
        Self { reader, chunk_size }
    }
}

impl<R: PointReader> Iterator for PointIterator<R> {
    type Item = Vec<Point>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut buffer = Vec::with_capacity(self.chunk_size);

        for _ in 0..self.chunk_size {
            match self.reader.next_point() {
                Ok(Some(p)) => buffer.push(p),
                Ok(None) => break,
                Err(e) => {
                    eprintln!("Error reading point: {}", e);
                    break;
                }
            }
        }

        if buffer.is_empty() {
            None
        } else {
            Some(buffer)
        }
    }
}
