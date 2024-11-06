use std::{error::Error, io::Write};

use cesiumtiles_gltf_json::{
    Accessor, AccessorType, Buffer, BufferView, BufferViewTarget, ComponentType, Gltf, Node, Scene,
};
use pcd_core::pointcloud::point::PointCloud;

pub fn write_glb<W: Write>(
    writer: W,
    points: &PointCloud,
) -> Result<(Gltf, Vec<u8>), Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    const VERTEX_BYTE_STRIDE: usize = 4 * 3; // 4byte(i32) x 3 (x, y, z)
    const COLOR_BYTE_STRIDE: usize = 2 * 4; // 2byte(u16) x 3 (r, g, b)

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; (VERTEX_BYTE_STRIDE + COLOR_BYTE_STRIDE)];

    for point in &points.points {
        let x = point.x.to_le_bytes();
        let y = point.y.to_le_bytes();
        let z = point.z.to_le_bytes();
        let r = point.color.r.to_le_bytes();
        let g = point.color.g.to_le_bytes();
        let b = point.color.b.to_le_bytes();

        buffer[0..4].copy_from_slice(&x);
        buffer[4..8].copy_from_slice(&y);
        buffer[8..12].copy_from_slice(&z);
        buffer[12..14].copy_from_slice(&r);
        buffer[14..16].copy_from_slice(&g);
        buffer[16..18].copy_from_slice(&b);

        bin_content.extend_from_slice(&buffer);
    }

    let len_vertices = bin_content.len() - buffer_offset;

    gltf_buffer_views.push(BufferView {
        name: Some("vertices".to_string()),
        byte_offset: buffer_offset as u32,
        byte_length: len_vertices as u32,
        byte_stride: Some(VERTEX_BYTE_STRIDE as u8),
        target: Some(BufferViewTarget::ArrayBuffer),
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("positions".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::Float,
        count: points.points.len() as u32,
        min: Some(points.metadata.bounding_volume.min.to_vec()),
        max: Some(points.metadata.bounding_volume.max.to_vec()),
        type_: AccessorType::Vec3,
        ..Default::default()
    });

    let gltf_buffers = {
        let mut buffers = vec![];
        if !bin_content.is_empty() {
            buffers.push(Buffer {
                byte_length: bin_content.len() as u32,
                ..Default::default()
            });
        }
        buffers
    };

    let gltf = Gltf {
        scenes: vec![Scene {
            nodes: Some(vec![0]),
            ..Default::default()
        }],
        nodes: vec![Node {
            mesh: Some(0),
            ..Default::default()
        }],
        accessors: gltf_accessors,
        buffer_views: gltf_buffer_views,
        buffers: gltf_buffers,
        ..Default::default()
    };

    cesiumtiles_gltf::glb::Glb {
        json: serde_json::to_vec(&gltf).unwrap().into(),
        bin: Some(bin_content.clone().into()),
    }
    .to_writer_with_alignment(writer, 8)?;

    Ok((gltf, bin_content))
}
