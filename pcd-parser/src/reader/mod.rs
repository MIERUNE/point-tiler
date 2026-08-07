pub mod csv;
pub mod las;

use pcd_core::pointcloud::point::Point;
use std::io;

pub trait PointReader {
    fn next_point(&mut self) -> io::Result<Option<Point>>;
}

/// Selects which optional point attributes a reader should populate.
/// A field set to `false` is stored as `None` and never written to GLB metadata.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AttributeSelection {
    pub intensity: bool,
    pub return_number: bool,
    pub classification: bool,
    pub scanner_channel: bool,
    pub scan_angle: bool,
    pub user_data: bool,
    pub point_source_id: bool,
    pub gps_time: bool,
}
