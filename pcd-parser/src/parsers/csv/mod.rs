use std::{collections::HashMap, error::Error, path::PathBuf};

use csv::ReaderBuilder;

use pcd_core::pointcloud::point::{Metadata, Point, PointAttributes, PointCloud};

use super::{Parser, ParserProvider};

pub struct CsvParserProvider {
    pub filenames: Vec<PathBuf>,
}

impl ParserProvider for CsvParserProvider {
    fn get_parser(&self) -> Box<dyn Parser> {
        Box::new(CsvParser {
            filenames: self.filenames.clone(),
        })
    }
}

pub struct CsvParser {
    pub filenames: Vec<PathBuf>,
}

impl Parser for CsvParser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>> {
        let start = std::time::Instant::now();
        let mut reader = ReaderBuilder::new()
            .has_headers(true)
            .from_path(&self.filenames[0])
            .unwrap();
        println!("Read CSV time: {:?}", start.elapsed());

        let headers = reader.headers().unwrap();
        let has_headers = !headers.iter().all(|h| h.trim().is_empty());

        let field_mapping = create_field_mapping(&headers, has_headers).unwrap();

        let mut points = Vec::new();

        let start = std::time::Instant::now();
        for record in reader.records() {
            let record: csv::StringRecord = record.unwrap();

            let x_str = get_field_value(&record, &field_mapping, "x").ok_or("Missing 'x' field")?;
            let y_str = get_field_value(&record, &field_mapping, "y").ok_or("Missing 'y' field")?;
            let z_str = get_field_value(&record, &field_mapping, "z").ok_or("Missing 'z' field")?;

            let x: f64 = x_str
                .parse()
                .map_err(|e| format!("Failed to parse 'x': {}", e))?;
            let y: f64 = y_str
                .parse()
                .map_err(|e| format!("Failed to parse 'y': {}", e))?;
            let z: f64 = z_str
                .parse()
                .map_err(|e| format!("Failed to parse 'z': {}", e))?;

            let attributes = PointAttributes {
                intensity: parse_optional_field(&record, &field_mapping, "intensity")?,
                return_number: parse_optional_field(&record, &field_mapping, "return_number")?,
                classification: get_field_value(&record, &field_mapping, "classification")
                    .map(|v| v.to_string()),
                scanner_channel: parse_optional_field(&record, &field_mapping, "scanner_channel")?,
                scan_angle: parse_optional_field(&record, &field_mapping, "scan_angle")?,
                user_data: parse_optional_field(&record, &field_mapping, "user_data")?,
                point_source_id: parse_optional_field(&record, &field_mapping, "point_source_id")?,
                gps_time: parse_optional_field(&record, &field_mapping, "gps_time")?,
                r: parse_optional_field(&record, &field_mapping, "r")?,
                g: parse_optional_field(&record, &field_mapping, "g")?,
                b: parse_optional_field(&record, &field_mapping, "b")?,
            };

            let point = Point {
                x,
                y,
                z,
                attributes,
            };

            points.push(point);
        }

        // todo: メタデータ取り込み部分を作る
        let metadata = Metadata {
            bounding_volume: Default::default(),
            coordinate_system_wkt: "PROJCS[\"JGD2011 / Japan Plane Rectangular CS VII\",...]"
                .to_string(),
            scale: [0.001, 0.001, 0.001],
            offset: [0.0, 0.0, 0.0],
            other: HashMap::new(),
        };

        let point_cloud = PointCloud { points, metadata };

        println!("Parse CSV time: {:?}", start.elapsed());
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
        record.get(index)
    } else {
        None
    }
}

fn parse_optional_field<T: std::str::FromStr>(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<Option<T>, Box<dyn Error>> {
    if let Some(value_str) = get_field_value(record, field_mapping, field_name) {
        if value_str.trim().is_empty() {
            Ok(None)
        } else {
            let value = value_str
                .parse::<T>()
                .map_err(|_| format!("Failed to parse '{}'", field_name))?;
            Ok(Some(value))
        }
    } else {
        Ok(None)
    }
}
