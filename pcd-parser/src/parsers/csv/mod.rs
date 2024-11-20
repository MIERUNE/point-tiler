use std::{collections::HashMap, error::Error, path::PathBuf};

use csv::ReaderBuilder;

use pcd_core::pointcloud::point::{Color, Point, PointAttributes, PointCloud};
use projection_transform::crs::EpsgCode;

use super::{Parser, ParserProvider};

pub struct CsvParserProvider {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl ParserProvider for CsvParserProvider {
    fn get_parser(&self) -> Box<dyn Parser> {
        Box::new(CsvParser {
            filenames: self.filenames.clone(),
            epsg: self.epsg,
        })
    }
}

pub struct CsvParser {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl Parser for CsvParser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>> {
        let mut reader = ReaderBuilder::new()
            .has_headers(true)
            .from_path(&self.filenames[0])
            .unwrap();

        let headers = reader.headers().unwrap();
        let has_headers = !headers.iter().all(|h| h.trim().is_empty());

        let field_mapping = create_field_mapping(headers, has_headers).unwrap();

        let mut reader = ReaderBuilder::new()
            .has_headers(true)
            .from_path(&self.filenames[0])
            .unwrap();
        let mut points = Vec::new();
        {
            for record in reader.records() {
                let record: csv::StringRecord = record.unwrap();

                let x_str =
                    get_field_value(&record, &field_mapping, "x").ok_or("Missing 'x' field")?;
                let y_str =
                    get_field_value(&record, &field_mapping, "y").ok_or("Missing 'y' field")?;
                let z_str =
                    get_field_value(&record, &field_mapping, "z").ok_or("Missing 'z' field")?;

                let x: f64 = x_str
                    .parse()
                    .map_err(|e| format!("Failed to parse 'x': {}", e))?;
                let y: f64 = y_str
                    .parse()
                    .map_err(|e| format!("Failed to parse 'y': {}", e))?;
                let z: f64 = z_str
                    .parse()
                    .map_err(|e| format!("Failed to parse 'z': {}", e))?;

                let r = parse_optional_field(&record, &field_mapping, "r")
                    .or_else(|| parse_optional_field(&record, &field_mapping, "red"))
                    .unwrap_or("65535".to_string())
                    .parse::<f64>()
                    .map_err(|e| format!("Failed to parse 'r': {}", e))?
                    .floor() as u16;

                let g = parse_optional_field(&record, &field_mapping, "g")
                    .or_else(|| parse_optional_field(&record, &field_mapping, "green"))
                    .unwrap_or("65535".to_string())
                    .parse::<f64>()
                    .map_err(|e| format!("Failed to parse 'g': {}", e))?
                    .floor() as u16;

                let b = parse_optional_field(&record, &field_mapping, "b")
                    .or_else(|| parse_optional_field(&record, &field_mapping, "blue"))
                    .unwrap_or("65535".to_string())
                    .parse::<f64>()
                    .map_err(|e| format!("Failed to parse 'b': {}", e))?
                    .floor() as u16;

                let color = Color { r, g, b };

                // TODO: 将来的に実装する
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

                let point = Point {
                    x,
                    y,
                    z,
                    color,
                    attributes,
                };

                points.push(point);
            }
        }

        let point_cloud = PointCloud::new(points, self.epsg);

        Ok(point_cloud)
    }
}

fn create_field_mapping(
    headers: &csv::StringRecord,
    has_headers: bool,
) -> Result<HashMap<String, usize>, Box<dyn Error>> {
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
            let normalized_header = header.to_lowercase().replace("_", "").replace("-", "");

            for attr_name in &attribute_names {
                let normalized_attr = attr_name.to_lowercase().replace("_", "").replace("-", "");
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
