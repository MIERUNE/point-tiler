use std::{collections::HashMap, error::Error, path::PathBuf};

use csv::ReaderBuilder;

use pcd_core::pointcloud::point::{Color, EpsgCode, Point, PointAttributes, PointCloud};

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

fn parse_optional_numeric<T>(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<Option<T>, Box<dyn Error>>
where
    T: TryFrom<i64>,
    <T as TryFrom<i64>>::Error: Error + Send + Sync + 'static,
{
    match parse_optional_field(record, field_mapping, field_name) {
        Some(v) => {
            let parsed = v.parse::<f64>()?.floor() as i64;
            Ok(Some(T::try_from(parsed)?))
        }
        None => Ok(None),
    }
}

fn parse_optional_f32(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<Option<f32>, Box<dyn Error>> {
    match parse_optional_field(record, field_mapping, field_name) {
        Some(v) => Ok(Some(v.parse::<f32>()?)),
        None => Ok(None),
    }
}

fn parse_optional_f64(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<Option<f64>, Box<dyn Error>> {
    match parse_optional_field(record, field_mapping, field_name) {
        Some(v) => Ok(Some(v.parse::<f64>()?)),
        None => Ok(None),
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

                let attributes = PointAttributes {
                    intensity: parse_optional_numeric::<u16>(&record, &field_mapping, "intensity")?,
                    return_number: parse_optional_numeric::<u8>(
                        &record,
                        &field_mapping,
                        "return_number",
                    )?,
                    classification: parse_optional_numeric::<u8>(
                        &record,
                        &field_mapping,
                        "classification",
                    )?,
                    scanner_channel: parse_optional_numeric::<u8>(
                        &record,
                        &field_mapping,
                        "scanner_channel",
                    )?,
                    scan_angle: parse_optional_f32(&record, &field_mapping, "scan_angle")?,
                    user_data: parse_optional_numeric::<u8>(&record, &field_mapping, "user_data")?,
                    point_source_id: parse_optional_numeric::<u16>(
                        &record,
                        &field_mapping,
                        "point_source_id",
                    )?,
                    gps_time: parse_optional_f64(&record, &field_mapping, "gps_time")?,
                };

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
        record.get(index)
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
