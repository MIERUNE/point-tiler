use std::{collections::HashMap, error::Error, io::Write};

use byteorder::{ByteOrder as _, LittleEndian};
use cesiumtiles_gltf_json::{
    Accessor, AccessorType, Buffer, BufferView, BufferViewTarget, ComponentType, Gltf, Mesh,
    MeshPrimitive, Node, Scene,
};
use pcd_core::pointcloud::point::PointCloud;

pub fn quantize_unsigned_norm(value: f32, bits: i32) -> i32 {
    let max_value = (1i32 << bits) - 1i32;
    let scale = max_value as f32;

    let clamped = value.clamp(0.0, 1.0);

    (clamped * scale + 0.5) as i32
}

fn rcp(value: f32) -> f32 {
    if value != 0.0 {
        1.0 / value
    } else {
        0.0
    }
}

pub fn generate_glb<'a>(
    points: PointCloud,
) -> Result<cesiumtiles_gltf::glb::Glb<'a>, Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    // TODO: カラーが存在しないデータに対応
    const BYTE_STRIDE: usize = (2 * 3 + 2) + (2 * 3 + 2);

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let scale = points.metadata.scale;
    let offset = points.metadata.offset;

    let mut quantized_position_max = [u16::MIN; 3];
    let mut quantized_position_min = [u16::MAX; 3];

    let bits = 14;
    let common_scale =
        points
            .iter_with_raw_coords()
            .fold(0f32, |result, (raw_x, raw_y, raw_z, _)| {
                result
                    .max(raw_x as f32 - offset[0] as f32)
                    .max(raw_y as f32 - offset[1] as f32)
                    .max(raw_z as f32 - offset[2] as f32)
            });
    let point_scale_inv = rcp(common_scale);

    // quantize
    for (raw_x, raw_y, raw_z, point) in points.iter_with_raw_coords() {
        let x = quantize_unsigned_norm((raw_x as f32 - offset[0] as f32) * point_scale_inv, bits)
            as u16;
        let y = quantize_unsigned_norm((raw_y as f32 - offset[1] as f32) * point_scale_inv, bits)
            as u16;
        let z = quantize_unsigned_norm((raw_z as f32 - offset[2] as f32) * point_scale_inv, bits)
            as u16;

        let r = point.color.r;
        let g = point.color.g;
        let b = point.color.b;

        quantized_position_max[0] = quantized_position_max[0].max(x);
        quantized_position_max[1] = quantized_position_max[1].max(y);
        quantized_position_max[2] = quantized_position_max[2].max(z);
        quantized_position_min[0] = quantized_position_min[0].min(x);
        quantized_position_min[1] = quantized_position_min[1].min(y);
        quantized_position_min[2] = quantized_position_min[2].min(z);

        LittleEndian::write_u16_into(&[x, y, z], &mut buffer[0..6]);
        buffer[6..8].copy_from_slice(&[0, 0]);
        LittleEndian::write_u16_into(&[r, g, b], &mut buffer[8..14]);
        buffer[14..16].copy_from_slice(&[0, 0]);

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
        component_type: ComponentType::UnsignedShort,
        count: points.points.len() as u32,
        min: Some(quantized_position_min.iter().map(|&x| x as f64).collect()),
        max: Some(quantized_position_max.iter().map(|&x| x as f64).collect()),
        type_: AccessorType::Vec3,
        normalized: true,
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("colors".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::UnsignedShort,
        byte_offset: 2 * 3 + 2,
        count: points.points.len() as u32,
        type_: AccessorType::Vec3,
        normalized: true,
        ..Default::default()
    });

    let gltf_meshes = vec![Mesh {
        primitives: vec![MeshPrimitive {
            attributes: HashMap::from_iter(vec![
                ("POSITION".to_string(), 0),
                ("COLOR_0".to_string(), 1),
            ]),
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

    let extensions_used = vec!["KHR_mesh_quantization".to_string()];
    let extensions_required = vec!["KHR_mesh_quantization".to_string()];

    let gltf = Gltf {
        scenes: vec![Scene {
            nodes: Some(vec![0]),
            ..Default::default()
        }],
        nodes: vec![Node {
            mesh: Some(0),
            translation: offset,
            scale,
            ..Default::default()
        }],
        meshes: gltf_meshes,
        accessors: gltf_accessors,
        buffer_views: gltf_buffer_views,
        buffers: gltf_buffers,
        extensions_used,
        extensions_required,
        ..Default::default()
    };

    Ok(cesiumtiles_gltf::glb::Glb {
        json: serde_json::to_vec(&gltf).unwrap().into(),
        bin: Some(bin_content.clone().into()),
    })
}
