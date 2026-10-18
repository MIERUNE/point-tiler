use std::{
    fs::File,
    io::{self, BufReader},
    path::{Path, PathBuf},
};

use las::Reader;
use las::point::{Classification, Format};
use pcd_core::pointcloud::point::{AttributeSelection, Color, Point, PointAttributes};

use super::PointReader;

/// Classification code that LAS 1.0-1.3 point formats (0-5) use for overlap points.
/// The `las` crate turns it into `Unclassified` plus `is_overlap`, so it is restored here.
const LEGACY_OVERLAP_CLASSIFICATION: u8 = 12;

pub struct LasPointReader {
    pub files: Vec<PathBuf>,
    pub current_file_index: usize,
    pub current_reader: Option<Reader>,
    pub selection: AttributeSelection,
    current_path: PathBuf,
    current_format: Format,
}

impl LasPointReader {
    pub fn new(files: Vec<PathBuf>, selection: AttributeSelection) -> io::Result<Self> {
        Ok(Self {
            files,
            current_file_index: 0,
            current_reader: None,
            selection,
            current_path: PathBuf::new(),
            current_format: Format::default(),
        })
    }

    pub fn estimate_processing_size(path: &PathBuf) -> u64 {
        Reader::from_path(path)
            .map(|reader| {
                let header = reader.header();
                header.number_of_points() * u64::from(header.point_format().len())
            })
            .unwrap_or_else(|_| path.metadata().map(|m| m.len()).unwrap_or(0))
    }

    /// Attributes that the given point format stores. GPS time is absent from
    /// formats 0 and 2, and the scanner channel exists only in formats 6-10.
    pub fn available_attributes(format: &Format) -> AttributeSelection {
        AttributeSelection {
            scanner_channel: format.is_extended,
            gps_time: format.has_gps_time,
            ..AttributeSelection::ALL
        }
    }

    /// Reads only the header of `path` and returns the attributes its points store.
    pub fn probe_attributes(path: &Path) -> io::Result<AttributeSelection> {
        let reader = Reader::from_path(path)
            .map_err(|e| io::Error::other(format!("{}: {}", path.display(), e)))?;
        Ok(Self::available_attributes(reader.header().point_format()))
    }

    pub fn open_next_file(&mut self) -> io::Result<()> {
        if self.current_file_index < self.files.len() {
            let path = self.files[self.current_file_index].clone();
            let file = File::open(&path)
                .map_err(|e| io::Error::new(e.kind(), format!("{}: {}", path.display(), e)))?;
            let reader = Reader::new(BufReader::new(file))
                .map_err(|e| io::Error::other(format!("{}: {}", path.display(), e)))?;

            let format = *reader.header().point_format();
            let missing = self
                .selection
                .difference(&Self::available_attributes(&format));
            if !missing.is_empty() {
                return Err(io::Error::other(format!(
                    "{}: LAS point format {} does not contain selected attribute(s): {}",
                    path.display(),
                    format.to_u8().unwrap_or_default(),
                    missing.names().join(", ")
                )));
            }

            self.current_reader = Some(reader);
            self.current_path = path;
            self.current_format = format;
            self.current_file_index += 1;
            Ok(())
        } else {
            self.current_reader = None;
            Ok(())
        }
    }

    /// Classification code as stored in the file. For point formats 0-5 the `las`
    /// crate reports code 12 (overlap) as `Unclassified` with `is_overlap` set.
    /// `is_overlap` alone is not enough: the crate tests only the low 4 bits, so
    /// code 28 also sets it while keeping classification 28.
    pub fn classification_code(las_point: &las::Point, format: &Format) -> u8 {
        if !format.is_extended
            && las_point.is_overlap
            && las_point.classification == Classification::Unclassified
        {
            LEGACY_OVERLAP_CLASSIFICATION
        } else {
            u8::from(las_point.classification)
        }
    }

    pub fn convert_las_point(
        las_point: las::Point,
        selection: AttributeSelection,
        format: &Format,
    ) -> Result<Point, String> {
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

        let classification = Self::classification_code(&las_point, format);

        let gps_time = if selection.gps_time {
            match las_point.gps_time {
                Some(t) if t.is_finite() => Some(t),
                Some(t) => return Err(format!("gps_time is not a finite number: {t}")),
                None => return Err("gps_time is missing".to_string()),
            }
        } else {
            None
        };

        let attributes = PointAttributes {
            intensity: selection.intensity.then_some(las_point.intensity),
            return_number: selection.return_number.then_some(las_point.return_number),
            classification: selection.classification.then_some(classification),
            scanner_channel: selection
                .scanner_channel
                .then_some(las_point.scanner_channel),
            scan_angle: selection.scan_angle.then_some(las_point.scan_angle),
            user_data: selection.user_data.then_some(las_point.user_data),
            point_source_id: selection
                .point_source_id
                .then_some(las_point.point_source_id),
            gps_time,
        };

        Ok(Point {
            x: las_point.x,
            y: las_point.y,
            z: las_point.z,
            color,
            attributes,
        })
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

            let selection = self.selection;
            let reader = self.current_reader.as_mut().unwrap();
            match reader.points().next() {
                Some(Ok(las_point)) => {
                    return Self::convert_las_point(las_point, selection, &self.current_format)
                        .map(Some)
                        .map_err(|e| {
                            io::Error::other(format!("{}: {}", self.current_path.display(), e))
                        });
                }
                Some(Err(e)) => {
                    return Err(io::Error::other(format!(
                        "{}: {}",
                        self.current_path.display(),
                        e
                    )));
                }
                None => {
                    self.current_reader = None;
                }
            }
        }
    }
}

pub struct PointIterator<R: PointReader> {
    pub reader: R,
    pub chunk_size: usize,
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

#[cfg(test)]
mod tests {
    use super::LasPointReader;
    use las::point::{Classification, Format};
    use pcd_core::pointcloud::point::AttributeSelection;

    fn sample_las_point() -> las::Point {
        las::Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 1234,
            return_number: 2,
            classification: Classification::Ground,
            scanner_channel: 3,
            scan_angle: -4.5,
            user_data: 17,
            point_source_id: 99,
            gps_time: Some(123.456),
            ..Default::default()
        }
    }

    #[test]
    fn convert_las_point_maps_selected_attributes() {
        let selection = AttributeSelection {
            intensity: true,
            return_number: true,
            classification: true,
            scanner_channel: true,
            scan_angle: true,
            user_data: true,
            point_source_id: true,
            gps_time: true,
        };
        let point = LasPointReader::convert_las_point(
            sample_las_point(),
            selection,
            &Format::new(6).unwrap(),
        )
        .unwrap();
        assert_eq!(point.attributes.intensity, Some(1234));
        assert_eq!(point.attributes.return_number, Some(2));
        assert_eq!(point.attributes.classification, Some(2));
        assert_eq!(point.attributes.scanner_channel, Some(3));
        assert_eq!(point.attributes.scan_angle, Some(-4.5));
        assert_eq!(point.attributes.user_data, Some(17));
        assert_eq!(point.attributes.point_source_id, Some(99));
        assert_eq!(point.attributes.gps_time, Some(123.456));
    }

    #[test]
    fn convert_las_point_drops_unselected_attributes() {
        let selection = AttributeSelection {
            intensity: true,
            classification: true,
            ..Default::default()
        };
        let point = LasPointReader::convert_las_point(
            sample_las_point(),
            selection,
            &Format::new(6).unwrap(),
        )
        .unwrap();
        assert_eq!(point.attributes.intensity, Some(1234));
        assert_eq!(point.attributes.classification, Some(2));
        assert_eq!(point.attributes.return_number, None);
        assert_eq!(point.attributes.scanner_channel, None);
        assert_eq!(point.attributes.scan_angle, None);
        assert_eq!(point.attributes.user_data, None);
        assert_eq!(point.attributes.point_source_id, None);
        assert_eq!(point.attributes.gps_time, None);
    }
}
