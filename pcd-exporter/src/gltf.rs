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

    const BYTE_STRIDE: usize = (4 * 3) + (3 + 1);

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let offset = points.metadata.offset;

    let mut min = [f64::MAX; 3];
    let mut max = [f64::MIN; 3];

    for (x, y, z, point) in points.iter() {
        LittleEndian::write_f32_into(
            &[
                (x - offset[0]) as f32,
                (y - offset[1]) as f32,
                (z - offset[2]) as f32,
            ],
            &mut buffer[0..12],
        );

        min[0] = min[0].min(x - offset[0]);
        min[1] = min[1].min(y - offset[1]);
        min[2] = min[2].min(z - offset[2]);
        max[0] = max[0].max(x - offset[0]);
        max[1] = max[1].max(y - offset[1]);
        max[2] = max[2].max(z - offset[2]);

        let rgb8 = point.to_rgb8_normalized();
        let r = rgb8[0];
        let g = rgb8[1];
        let b = rgb8[2];

        buffer[12..15].copy_from_slice(&[r, g, b]);
        buffer[15..16].copy_from_slice(&[0]);

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
        min: Some(min.to_vec()),
        max: Some(max.to_vec()),
        type_: AccessorType::Vec3,
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("colors".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::UnsignedByte,
        byte_offset: 4 * 3,
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

    let gltf = Gltf {
        scenes: vec![Scene {
            nodes: Some(vec![0]),
            ..Default::default()
        }],
        nodes: vec![Node {
            mesh: Some(0),
            translation: offset,
            ..Default::default()
        }],
        meshes: gltf_meshes,
        accessors: gltf_accessors,
        buffer_views: gltf_buffer_views,
        buffers: gltf_buffers,
        ..Default::default()
    };

    Ok(cesiumtiles_gltf::glb::Glb {
        json: serde_json::to_vec(&gltf).unwrap().into(),
        bin: Some(bin_content.into()),
    })
}

pub fn generate_quantized_glb<'a>(
    points: PointCloud,
) -> Result<cesiumtiles_gltf::glb::Glb<'a>, Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    // TODO: カラーが存在しないデータに対応
    const BYTE_STRIDE: usize = (2 * 3 + 2) + (3 + 1);

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let offset = points.metadata.offset;

    let mut quantized_position_max = [u16::MIN; 3];
    let mut quantized_position_min = [u16::MAX; 3];

    let bits = 16;
    let common_scale = points.iter().fold(0f32, |result, (x, y, z, _)| {
        result
            .max(x as f32 - offset[0] as f32)
            .max(y as f32 - offset[1] as f32)
            .max(z as f32 - offset[2] as f32)
    });
    let point_scale_inv = rcp(common_scale);

    // quantize
    for (raw_x, raw_y, raw_z, point) in points.iter() {
        let quantized_x =
            quantize_unsigned_norm((raw_x as f32 - offset[0] as f32) * point_scale_inv, bits)
                as u16;
        let quantized_y =
            quantize_unsigned_norm((raw_y as f32 - offset[1] as f32) * point_scale_inv, bits)
                as u16;
        let quantized_z =
            quantize_unsigned_norm((raw_z as f32 - offset[2] as f32) * point_scale_inv, bits)
                as u16;

        let rgb8 = point.to_rgb8_normalized();
        let r = rgb8[0];
        let g = rgb8[1];
        let b = rgb8[2];

        quantized_position_max[0] = quantized_position_max[0].max(quantized_x);
        quantized_position_max[1] = quantized_position_max[1].max(quantized_y);
        quantized_position_max[2] = quantized_position_max[2].max(quantized_z);
        quantized_position_min[0] = quantized_position_min[0].min(quantized_x);
        quantized_position_min[1] = quantized_position_min[1].min(quantized_y);
        quantized_position_min[2] = quantized_position_min[2].min(quantized_z);

        LittleEndian::write_u16_into(&[quantized_x, quantized_y, quantized_z], &mut buffer[0..6]);
        buffer[6..8].copy_from_slice(&[0, 0]);
        buffer[8..11].copy_from_slice(&[r, g, b]);
        buffer[11..12].copy_from_slice(&[0]);

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
        component_type: ComponentType::UnsignedByte,
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
            scale: [
                common_scale as f64,
                common_scale as f64,
                common_scale as f64,
            ],
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
        bin: Some(bin_content.into()),
    })
}
