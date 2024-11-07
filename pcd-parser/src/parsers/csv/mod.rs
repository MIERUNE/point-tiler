use std::{collections::HashMap, error::Error, path::PathBuf};

use csv::ReaderBuilder;

use pcd_core::pointcloud::point::{
    BoundingVolume, Color, Metadata, Point, PointAttributes, PointCloud,
};

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

pub static SCALE_FACTOR: f64 = 0.001;

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

        let field_mapping = create_field_mapping(headers, has_headers).unwrap();

        let mut bounding_volume = BoundingVolume {
            min: [f64::MAX, f64::MAX, f64::MAX],
            max: [f64::MIN, f64::MIN, f64::MIN],
        };
        let mut digits_x = 3;
        let mut digits_y = 3;
        let mut digits_z = 3;

        let mut point_count = 0;

        let start = std::time::Instant::now();
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

                bounding_volume.max[0] = bounding_volume.max[0].max(x);
                bounding_volume.max[1] = bounding_volume.max[1].max(y);
                bounding_volume.max[2] = bounding_volume.max[2].max(z);
                bounding_volume.min[0] = bounding_volume.min[0].min(x);
                bounding_volume.min[1] = bounding_volume.min[1].min(y);
                bounding_volume.min[2] = bounding_volume.min[2].min(z);

                for (value, digits) in [(x, &mut digits_x), (y, &mut digits_y), (z, &mut digits_z)]
                {
                    let value_str = format!("{:.7}", value);
                    if let Some(dot_index) = value_str.find('.') {
                        let fractional_part = &value_str[dot_index + 1..];
                        let fractional_part = fractional_part.trim_end_matches('0');
                        *digits = *digits.max(&mut fractional_part.len());
                    }
                }

                point_count += 1;
            }
        }

        let scale_x: f64 = format!("{:.*}", digits_x, 0.1_f64.powi(digits_x as i32)).parse()?;
        let scale_y: f64 = format!("{:.*}", digits_y, 0.1_f64.powi(digits_y as i32)).parse()?;
        let scale_z: f64 = format!("{:.*}", digits_z, 0.1_f64.powi(digits_z as i32)).parse()?;

        let min_x = bounding_volume.min[0];
        let min_y = bounding_volume.min[1];
        let min_z = bounding_volume.min[2];

        let offset_x = min_x;
        let offset_y = min_y;
        let offset_z = min_z;

        let metadata = Metadata {
            point_count,
            bounding_volume,
            coordinate_system_wkt: "PROJCS[\"JGD2011 / Japan Plane Rectangular CS VII\",...]"
                .to_string(),
            scale: [scale_x, scale_y, scale_z],
            offset: [offset_x, offset_y, offset_z],
            other: HashMap::new(),
        };
        println!("Calc bounding_volume time: {:?}", start.elapsed());

        let start = std::time::Instant::now();
        // TODO: 1度目のループで消費されてしまうので、再度読み込みを行っているが、改修が必要
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

                let color = Color {
                    r: parse_optional_field(&record, &field_mapping, "r")?.unwrap_or(0),
                    g: parse_optional_field(&record, &field_mapping, "g")?.unwrap_or(0),
                    b: parse_optional_field(&record, &field_mapping, "b")?.unwrap_or(0),
                };

                let attributes = PointAttributes {
                    intensity: parse_optional_field(&record, &field_mapping, "intensity")?,
                    return_number: parse_optional_field(&record, &field_mapping, "return_number")?,
                    classification: get_field_value(&record, &field_mapping, "classification")
                        .map(|v| v.to_string()),
                    scanner_channel: parse_optional_field(
                        &record,
                        &field_mapping,
                        "scanner_channel",
                    )?,
                    scan_angle: parse_optional_field(&record, &field_mapping, "scan_angle")?,
                    user_data: parse_optional_field(&record, &field_mapping, "user_data")?,
                    point_source_id: parse_optional_field(
                        &record,
                        &field_mapping,
                        "point_source_id",
                    )?,
                    gps_time: parse_optional_field(&record, &field_mapping, "gps_time")?,
                };

                let point = Point {
                    x: ((x - offset_x) / scale_x) as u32,
                    y: ((y - offset_y) / scale_y) as u32,
                    z: ((z - offset_z) / scale_z) as u32,
                    color,
                    attributes,
                };

                points.push(point);
            }
        }
        println!("Parse CSV time: {:?}", start.elapsed());

        let point_cloud = PointCloud { points, metadata };

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
