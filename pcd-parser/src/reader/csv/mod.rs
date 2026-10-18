use std::{
    collections::HashMap,
    error::Error,
    fs::File,
    io::{self, BufReader, Read, Seek, SeekFrom},
    path::{Path, PathBuf},
};

use csv::ReaderBuilder;
use pcd_core::pointcloud::point::{AttributeSelection, Color, Point, PointAttributes};

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
        record.get(index)
    } else {
        None
    }
}

/// Returns the trimmed cell of a selected attribute, or an error naming the
/// attribute when the cell is blank. Missing columns are rejected when the file is opened.
fn required_attribute<'a>(
    record: &'a csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<&'a str, String> {
    match get_field_value(record, field_mapping, field_name).map(str::trim) {
        Some(v) if !v.is_empty() => Ok(v),
        _ => Err(format!("'{field_name}' has no value")),
    }
}

/// Parses an integer-valued attribute. Values with a fractional part or outside
/// the range of `T` are rejected instead of being truncated.
fn parse_integer<T>(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
) -> Result<T, String>
where
    T: TryFrom<i64>,
{
    let v = required_attribute(record, field_mapping, field_name)?;
    let integer = match v.parse::<i64>() {
        Ok(i) => i,
        Err(_) => match v.parse::<f64>() {
            Ok(f) if f.is_finite() && f.fract() == 0.0 => f as i64,
            _ => return Err(format!("'{field_name}' is not an integer: {v}")),
        },
    };
    T::try_from(integer).map_err(|_| format!("'{field_name}' is out of range: {v}"))
}

fn parse_finite<T>(
    record: &csv::StringRecord,
    field_mapping: &HashMap<String, usize>,
    field_name: &str,
    is_finite: impl Fn(&T) -> bool,
) -> Result<T, String>
where
    T: std::str::FromStr,
{
    let v = required_attribute(record, field_mapping, field_name)?;
    match v.parse::<T>() {
        Ok(parsed) if is_finite(&parsed) => Ok(parsed),
        Ok(_) => Err(format!("'{field_name}' is not a finite number: {v}")),
        Err(_) => Err(format!("'{field_name}' is not a number: {v}")),
    }
}

/// Runs `parse` only when the attribute is selected, otherwise yields `None`.
fn parse_when<T>(
    selected: bool,
    parse: impl FnOnce() -> Result<T, String>,
) -> Result<Option<T>, String> {
    if selected {
        parse().map(Some)
    } else {
        Ok(None)
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
    pub selection: AttributeSelection,
    current_path: PathBuf,
}

impl CsvPointReader {
    pub fn new(files: Vec<PathBuf>, selection: AttributeSelection) -> io::Result<Self> {
        let mut reader = CsvPointReader {
            files,
            current_file_index: 0,
            current_reader: None,
            field_mapping: HashMap::new(),
            selection,
            current_path: PathBuf::new(),
        };

        reader.open_next_file()?;
        Ok(reader)
    }

    /// Opens `path` and builds the column mapping from its header row.
    fn open_with_mapping(path: &Path) -> io::Result<(csv::Reader<File>, HashMap<String, usize>)> {
        let with_path = |e: &dyn std::fmt::Display| format!("{}: {}", path.display(), e);
        let mut rdr = ReaderBuilder::new()
            .has_headers(true)
            .from_path(path)
            .map_err(|e| io::Error::other(with_path(&e)))?;

        let headers = rdr
            .headers()
            .map_err(|e| io::Error::other(with_path(&e)))?
            .clone();
        let has_headers = !headers.iter().all(|h| h.trim().is_empty());

        let mapping = create_field_mapping(&headers, has_headers)
            .map_err(|e| io::Error::other(with_path(&e)))?;
        Ok((rdr, mapping))
    }

    fn available_attributes(mapping: &HashMap<String, usize>) -> AttributeSelection {
        let has = |name| mapping.contains_key(name);
        AttributeSelection {
            intensity: has("intensity"),
            return_number: has("return_number"),
            classification: has("classification"),
            scanner_channel: has("scanner_channel"),
            scan_angle: has("scan_angle"),
            user_data: has("user_data"),
            point_source_id: has("point_source_id"),
            gps_time: has("gps_time"),
        }
    }

    /// Reads only the header row of `path` and returns the attribute columns it has.
    pub fn probe_attributes(path: &Path) -> io::Result<AttributeSelection> {
        let (_, mapping) = Self::open_with_mapping(path)?;
        Ok(Self::available_attributes(&mapping))
    }

    fn open_next_file(&mut self) -> io::Result<()> {
        if self.current_file_index < self.files.len() {
            let path = self.files[self.current_file_index].clone();
            self.current_file_index += 1;

            let (rdr, mapping) = Self::open_with_mapping(&path)?;

            let missing = self
                .selection
                .difference(&Self::available_attributes(&mapping));
            if !missing.is_empty() {
                return Err(io::Error::other(format!(
                    "{}: CSV header does not contain selected attribute column(s): {}",
                    path.display(),
                    missing.names().join(", ")
                )));
            }

            self.field_mapping = mapping;
            self.current_reader = Some(rdr);
            self.current_path = path;
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

        let sel = &self.selection;
        let attributes = PointAttributes {
            intensity: parse_when(sel.intensity, || {
                parse_integer::<u16>(record, &self.field_mapping, "intensity")
            })?,
            return_number: parse_when(sel.return_number, || {
                parse_integer::<u8>(record, &self.field_mapping, "return_number")
            })?,
            classification: parse_when(sel.classification, || {
                parse_integer::<u8>(record, &self.field_mapping, "classification")
            })?,
            scanner_channel: parse_when(sel.scanner_channel, || {
                parse_integer::<u8>(record, &self.field_mapping, "scanner_channel")
            })?,
            scan_angle: parse_when(sel.scan_angle, || {
                parse_finite(record, &self.field_mapping, "scan_angle", |v: &f32| {
                    v.is_finite()
                })
            })?,
            user_data: parse_when(sel.user_data, || {
                parse_integer::<u8>(record, &self.field_mapping, "user_data")
            })?,
            point_source_id: parse_when(sel.point_source_id, || {
                parse_integer::<u16>(record, &self.field_mapping, "point_source_id")
            })?,
            gps_time: parse_when(sel.gps_time, || {
                parse_finite(record, &self.field_mapping, "gps_time", |v: &f64| {
                    v.is_finite()
                })
            })?,
        };

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
                Ok(true) => {
                    return self.parse_point(&record).map(Some).map_err(|e| {
                        // csv may leave the LF of a CRLF terminator (and
                        // empty lines) at the recorded start position, so
                        // count them to report the physical line. If the file
                        // cannot be reread, keep the line csv recorded.
                        let line = record.position().map_or(0, |position| {
                            let count_newlines = || -> io::Result<u64> {
                                let mut source = BufReader::new(File::open(&self.current_path)?);
                                source.seek(SeekFrom::Start(position.byte()))?;
                                let mut newlines = 0;
                                for byte in source.bytes() {
                                    match byte? {
                                        b'\n' => newlines += 1,
                                        b'\r' => {}
                                        _ => break,
                                    }
                                }
                                Ok(newlines)
                            };
                            position.line() + count_newlines().unwrap_or(0)
                        });
                        io::Error::other(format!("{}:{}: {}", self.current_path.display(), line, e))
                    });
                }
                Ok(false) => {
                    self.current_reader = None;
                }
                Err(e) => {
                    return Err(io::Error::other(format!(
                        "{}: {}",
                        self.current_path.display(),
                        e
                    )));
                }
            }
        }
    }
}
