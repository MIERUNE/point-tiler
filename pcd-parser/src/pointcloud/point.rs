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
    pub r: Option<u16>,
    pub g: Option<u16>,
    pub b: Option<u16>,
}

#[derive(Debug, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub attributes: PointAttributes,
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Point>,
    pub metadata: Metadata,
}

#[derive(Debug, Clone)]
pub struct Metadata {
    pub coordinate_system_wkt: String,
    pub scale: [f64; 3],
    pub offset: [f64; 3],
    pub max_xyz: [f64; 3],
    pub min_xyz: [f64; 3],
    pub other: HashMap<String, String>,
}
