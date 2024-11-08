use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct PointAttributes {
    pub intensity: Option<u16>,
    pub return_number: Option<u8>,
    pub classification: Option<String>,
    pub scanner_channel: Option<u8>,
    pub scan_angle: Option<f32>,
    pub user_data: Option<u8>,
    pub point_source_id: Option<u16>,
    pub gps_time: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct Color {
    pub r: u16,
    pub g: u16,
    pub b: u16,
}

// The coordinates are expressed in i32 format
// The actual coordinates are calculated as follows, based on the combination of scale and offset
// x = (x * scale[0]) + offset[0]
#[derive(Debug, Clone)]
pub struct Point {
    pub x: u32,
    pub y: u32,
    pub z: u32,
    pub color: Color,
    pub attributes: PointAttributes,
}

impl Point {
    pub fn original_coordinates(&self, scale: [f64; 3], offset: [f64; 3]) -> [f64; 3] {
        [
            (self.x as f64 * scale[0]) + offset[0],
            (self.y as f64 * scale[1]) + offset[1],
            (self.z as f64 * scale[2]) + offset[2],
        ]
    }
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Point>,
    pub metadata: Metadata,
}

impl PointCloud {
    pub fn iter(&self) -> impl Iterator<Item = &Point> {
        self.points.iter()
    }
}

// This represents the maximum and minimum values of the original coordinate values obtained by combining the scale and offset.
#[derive(Debug, Clone, Default)]
pub struct BoundingVolume {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct Metadata {
    pub point_count: usize,
    pub bounding_volume: BoundingVolume,
    pub coordinate_system_wkt: String,
    pub scale: [f64; 3],
    pub offset: [f64; 3],
    pub other: HashMap<String, String>,
}
