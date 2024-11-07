use std::{collections::HashMap, error::Error, io::Write};

use byteorder::{ByteOrder as _, LittleEndian};
use cesiumtiles_gltf_json::{
    Accessor, AccessorType, Buffer, BufferView, BufferViewTarget, ComponentType, Gltf, Mesh,
    MeshPrimitive, Node, Scene,
};
use pcd_core::pointcloud::point::PointCloud;

pub fn write_glb<W: Write>(
    writer: W,
    points: &PointCloud,
) -> Result<(Gltf, Vec<u8>), Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    const BYTE_STRIDE: usize = 4 * 6;

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let mut position_max = [f64::MIN; 3];
    let mut position_min = [f64::MAX; 3];

    let scale = points.metadata.scale;
    let offset = points.metadata.offset;

    // TODO: scaleとoffsetを使って、u32のままでバイナリに書き込む方法（KHR_mesh_quantizationは利用できるか？）
    // TODO: color情報を書き込む方法

    for point in &points.points {
        let x = (point.x as f32 * scale[0] as f32 + offset[0] as f32).to_bits();
        let y = (point.y as f32 * scale[1] as f32 + offset[1] as f32).to_bits();
        let z = (point.z as f32 * scale[2] as f32 + offset[2] as f32).to_bits();
        let r = point.color.r as u32;
        let g = point.color.g as u32;
        let b = point.color.b as u32;

        position_max[0] = position_max[0].max(f32::from_bits(x) as f64);
        position_max[1] = position_max[1].max(f32::from_bits(y) as f64);
        position_max[2] = position_max[2].max(f32::from_bits(z) as f64);
        position_min[0] = position_min[0].min(f32::from_bits(x) as f64);
        position_min[1] = position_min[1].min(f32::from_bits(y) as f64);
        position_min[2] = position_min[2].min(f32::from_bits(z) as f64);

        LittleEndian::write_u32_into(&[x, y, z, r, g, b], &mut buffer);
        bin_content.write_all(&buffer)?;
    }

    let byte_length = bin_content.len() - buffer_offset;

    gltf_buffer_views.push(BufferView {
        name: Some("vertices".to_string()),
        byte_offset: buffer_offset as u32,
        byte_length: byte_length as u32,
        byte_stride: Some(BYTE_STRIDE as u8),
        target: Some(BufferViewTarget::ArrayBuffer),
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("positions".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::Float,
        count: points.points.len() as u32,
        min: Some(position_min.to_vec()),
        max: Some(position_max.to_vec()),
        type_: AccessorType::Vec3,
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("colors".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::UnsignedInt,
        byte_offset: 4 * 3,
        count: points.points.len() as u32,
        type_: AccessorType::Vec3,
        ..Default::default()
    });

    let gltf_meshes = vec![Mesh {
        primitives: vec![MeshPrimitive {
            attributes: HashMap::from_iter(vec![("POSITION".to_string(), 0)]),
            mode: cesiumtiles_gltf_json::PrimitiveMode::Points,
            ..Default::default()
        }],
        ..Default::default()
    }];

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
            translation: points.metadata.offset,
            scale: points.metadata.scale,
            ..Default::default()
        }],
        meshes: gltf_meshes,
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
