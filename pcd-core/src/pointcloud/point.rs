use std::collections::HashMap;

use projection_transform::crs::EpsgCode;

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

// LAS data coordinates are expressed in u32 format
// The actual coordinates are calculated based on a combination of scale and offset, as follows
// x = (x * scale[0]) + offset[0]
// TODO: カラーが存在しないデータに対応
#[derive(Debug, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub color: Color,
    pub attributes: PointAttributes,
}

#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<Point>,
    pub metadata: Metadata,
}

impl PointCloud {
    pub fn new(points: Vec<Point>, epsg: EpsgCode) -> Self {
        let mut bounding_volume = BoundingVolume {
            min: [f64::MAX, f64::MAX, f64::MAX],
            max: [f64::MIN, f64::MIN, f64::MIN],
        };
        let mut digits_x = 1;
        let mut digits_y = 1;
        let mut digits_z = 1;

        let mut point_count = 0;

        for point in &points {
            bounding_volume.max[0] = bounding_volume.max[0].max(point.x);
            bounding_volume.max[1] = bounding_volume.max[1].max(point.y);
            bounding_volume.max[2] = bounding_volume.max[2].max(point.z);
            bounding_volume.min[0] = bounding_volume.min[0].min(point.x);
            bounding_volume.min[1] = bounding_volume.min[1].min(point.y);
            bounding_volume.min[2] = bounding_volume.min[2].min(point.z);

            for (value, digits) in [
                (point.x, &mut digits_x),
                (point.y, &mut digits_y),
                (point.z, &mut digits_z),
            ] {
                let value_str = format!("{:.7}", value);
                if let Some(dot_index) = value_str.find('.') {
                    let fractional_part = &value_str[dot_index + 1..];
                    let fractional_part = fractional_part.trim_end_matches('0');
                    *digits = *digits.max(&mut fractional_part.len());
                }
            }

            point_count += 1;
        }

        let max_digits = digits_x.max(digits_y).max(digits_z);

        let scale_x: f64 = format!("{:.*}", max_digits, 0.1_f64.powi(max_digits as i32))
            .parse()
            .unwrap();
        let scale_y: f64 = format!("{:.*}", max_digits, 0.1_f64.powi(max_digits as i32))
            .parse()
            .unwrap();
        let scale_z: f64 = format!("{:.*}", max_digits, 0.1_f64.powi(max_digits as i32))
            .parse()
            .unwrap();

        let min_x = bounding_volume.min[0];
        let min_y = bounding_volume.min[1];
        let min_z = bounding_volume.min[2];

        let offset_x = min_x;
        let offset_y = min_y;
        let offset_z = min_z;

        let metadata = Metadata {
            point_count,
            bounding_volume,
            epsg,
            scale: [scale_x, scale_y, scale_z],
            offset: [offset_x, offset_y, offset_z],
            other: HashMap::new(),
        };

        PointCloud { points, metadata }
    }

    pub fn iter(&self) -> impl Iterator<Item = (f64, f64, f64, &Point)> {
        self.points
            .iter()
            .map(|point| (point.x, point.y, point.z, point))
    }

    pub fn iter_with_scaled_coords(&self) -> impl Iterator<Item = (u32, u32, u32, &Point)> + '_ {
        let scale = self.metadata.scale;
        let offset = self.metadata.offset;
        self.points.iter().map(move |point| {
            let x = ((point.x - offset[0]) / scale[0]) as u32;
            let y = ((point.y - offset[1]) / scale[1]) as u32;
            let z = ((point.z - offset[2]) / scale[2]) as u32;
            (x, y, z, point)
        })
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
    pub epsg: EpsgCode,
    pub scale: [f64; 3],
    pub offset: [f64; 3],
    pub other: HashMap<String, String>,
}
