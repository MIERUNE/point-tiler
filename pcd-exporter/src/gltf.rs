use std::{error::Error, io::Write};

use pcd_core::pointcloud::point::PointCloud;

pub fn write_gltf<W: Write>(writer: W, points: &PointCloud) -> Result<(), Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    let vertex_byte_stride = 4 * 3; // 4 bytes per float, 3 floats per vertex
    let color_byte_stride = 4 * 3; // 4 bytes per float, 3 floats per color

    Ok(())
}
