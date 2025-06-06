use std::{collections::HashMap, error::Error, fs::File, io, path::PathBuf};

use csv::ReaderBuilder;
use pcd_core::pointcloud::point::{Color, Point, PointAttributes};

use super::PointReader;

fn create_field_mapping(
    headers: &csv::StringRecord,
    has_headers: bool,
) -> Result<HashMap<String, usize>, Box<dyn Error + Send + Sync>> {
    let mut mapping = HashMap::new();

    let attribute_names = vec![
        "x",
        "y",
        "z",
        "intensity",
        "return_number",
        "classification",
        "scanner_channel",
        "scan_angle",
        "user_data",
        "point_source_id",
        "gps_time",
        "r",
        "g",
        "b",
        "red",
        "green",
        "blue",
    ];

    if has_headers {
        for (index, header) in headers.iter().enumerate() {
            let normalized_header = header.to_lowercase().replace(['_', '-'], "");

            for attr_name in &attribute_names {
                let normalized_attr = attr_name.to_lowercase().replace(['_', '-'], "");
                if normalized_header == normalized_attr {
                    mapping.insert(attr_name.to_string(), index);
                    break;
                }
            }
        }
    } else {
        for (index, attr_name) in attribute_names.iter().enumerate() {
            mapping.insert(attr_name.to_string(), index);
        }
    }

    for attr_name in &["x", "y", "z"] {
        if !mapping.contains_key(*attr_name) {
            return Err(format!(
                "Required attribute '{}' is missing in CSV headers or mapping.",
                attr_name
            )
            .into());
        }
    }

    Ok(mapping)
}

fn get_field_value<'a>(
    record: &'a csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Option<&'a str> {
    if let Some(&index) = field_mapping.get(field_name) {
        let value = record.get(index);
        value
    } else {
        None
    }
}

fn parse_optional_field(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Option<String> {
    if let Some(value_str) = get_field_value(record, field_mapping, field_name) {
        if value_str.trim().is_empty() {
            None
        } else {
            Some(value_str.to_string())
        }
    } else {
        None
    }
}

pub struct CsvPointReader {
    pub files: Vec<PathBuf>,
    pub current_file_index: usize,
    pub current_reader: Option<csv::Reader<File>>,
    pub field_mapping: HashMap<String, usize>,
}

impl CsvPointReader {
    pub fn new(files: Vec<PathBuf>) -> io::Result<Self> {
        let mut reader = CsvPointReader {
            files,
            current_file_index: 0,
            current_reader: None,
            field_mapping: HashMap::new(),
        };

        reader.open_next_file()?;
        Ok(reader)
    }

    fn open_next_file(&mut self) -> io::Result<()> {
        if self.current_file_index < self.files.len() {
            let path = &self.files[self.current_file_index];
            self.current_file_index += 1;

            let mut rdr = ReaderBuilder::new().has_headers(true).from_path(path)?;

            let headers = rdr.headers()?.clone();
            let has_headers = !headers.iter().all(|h| h.trim().is_empty());

            let mapping = create_field_mapping(&headers, has_headers)
                .map_err(io::Error::other)?;

            self.field_mapping = mapping;
            self.current_reader = Some(rdr);
            Ok(())
        } else {
            self.current_reader = None;
            Ok(())
        }
    }

    fn parse_point(&self, record: &csv::StringRecord) -> Result<Point, Box<dyn Error>> {
        let x_str = get_field_value(record, &self.field_mapping, "x").ok_or("Missing 'x' field")?;
        let y_str = get_field_value(record, &self.field_mapping, "y").ok_or("Missing 'y' field")?;
        let z_str = get_field_value(record, &self.field_mapping, "z").ok_or("Missing 'z' field")?;

        let x: f64 = x_str.parse()?;
        let y: f64 = y_str.parse()?;
        let z: f64 = z_str.parse()?;

        let r = parse_optional_field(record, &self.field_mapping, "r")
            .or_else(|| parse_optional_field(record, &self.field_mapping, "red"))
            .unwrap_or("65535".to_string())
            .parse::<f64>()?
            .floor() as u16;

        let g = parse_optional_field(record, &self.field_mapping, "g")
            .or_else(|| parse_optional_field(record, &self.field_mapping, "green"))
            .unwrap_or("65535".to_string())
            .parse::<f64>()?
            .floor() as u16;

        let b = parse_optional_field(record, &self.field_mapping, "b")
            .or_else(|| parse_optional_field(record, &self.field_mapping, "blue"))
            .unwrap_or("65535".to_string())
            .parse::<f64>()?
            .floor() as u16;

        let color = Color { r, g, b };

        let attributes = PointAttributes {
            intensity: None,
            return_number: None,
            classification: None,
            scanner_channel: None,
            scan_angle: None,
            user_data: None,
            point_source_id: None,
            gps_time: None,
        };
        // TODO: To be implemented in the future
        // let attributes = PointAttributes {
        //     intensity: parse_optional_field(&record, &field_mapping, "intensity")
        //         .unwrap_or(None),
        //     return_number: parse_optional_field(&record, &field_mapping, "return_number")
        //         .unwrap_or(None),
        //     classification: get_field_value(&record, &field_mapping, "classification")
        //         .map(|v| v.to_string()),
        //     scanner_channel: parse_optional_field(
        //         &record,
        //         &field_mapping,
        //         "scanner_channel",
        //     )
        //     .unwrap_or(None),
        //     scan_angle: parse_optional_field(&record, &field_mapping, "scan_angle")
        //         .unwrap_or(None),
        //     user_data: parse_optional_field(&record, &field_mapping, "user_data")
        //         .unwrap_or(None),
        //     point_source_id: parse_optional_field(
        //         &record,
        //         &field_mapping,
        //         "point_source_id",
        //     )
        //     .unwrap_or(None),
        //     gps_time: parse_optional_field(&record, &field_mapping, "gps_time")
        //         .unwrap_or(None),
        // };

        Ok(Point {
            x,
            y,
            z,
            color,
            attributes,
        })
    }
}

impl PointReader for CsvPointReader {
    fn next_point(&mut self) -> io::Result<Option<Point>> {
        loop {
            if self.current_reader.is_none() {
                self.open_next_file()?;
                if self.current_reader.is_none() {
                    return Ok(None);
                }
            }

            let reader = self.current_reader.as_mut().unwrap();
            let mut record = csv::StringRecord::new();

            match reader.read_record(&mut record) {
                Ok(true) => match self.parse_point(&record) {
                    Ok(p) => return Ok(Some(p)),
                    Err(e) => {
                        eprintln!("Error parsing CSV point: {}", e);
                        return Err(io::Error::other(format!("{}", e)));
                    }
                },
                Ok(false) => {
                    self.current_reader = None;
                }
                Err(e) => {
                    eprintln!("Error reading CSV record: {}", e);
                    return Err(io::Error::other(e));
                }
            }
        }
    }
}
