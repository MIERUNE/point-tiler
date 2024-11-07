use std::{collections::HashMap, error::Error, io::Write};

use byteorder::{ByteOrder as _, LittleEndian};
use cesiumtiles_gltf_json::{
    Accessor, AccessorType, Buffer, BufferView, BufferViewTarget, ComponentType, Gltf, Mesh,
    MeshPrimitive, Node, Scene,
};
use pcd_core::pointcloud::point::PointCloud;

fn quantize(val: f64, min: f64, max: f64) -> u16 {
    if val <= min {
        0
    } else if val >= max {
        u16::MAX
    } else {
        // quantizedValue = round(((originalValue - minValue) / (maxValue - minValue)) * maxQuantizedValue)
        // ((val - min) / (max - min) * u16::MAX as f64).round() as u16
        let normalized = (val - min) / (max - min);
        (normalized * u16::MAX as f64).round() as u16
    }
}

fn quantize_half(value: f32) -> u16 {
    let bits = value.to_bits();

    let sign = ((bits >> 16) & 0x8000) as u16;

    let em = bits & 0x7fff_ffff;

    let mut h = ((em - (112 << 23) + (1 << 12)) >> 13) as u16;

    if em < (113 << 23) {
        h = 0;
    }

    if em >= (143 << 23) {
        h = 0x7c00;
    }

    if em > (255 << 23) {
        h = 0x7e00;
    }

    sign | h
}

/// 浮動小数点数を [0, 1] の範囲で N ビットの符号なし整数に量子化する関数
pub fn quantize_unorm(value: f32, bits: i32) -> i32 {
    // スケールファクターを計算（最大値を取得）
    let max_value = (1i32 << bits) - 1i32;
    let scale = max_value as f32;

    // 入力値を [0.0, 1.0] の範囲にクランプ
    let clamped = if value < 0.0 {
        0.0
    } else if value > 1.0 {
        1.0
    } else {
        value
    };

    // スケールして四捨五入し、整数に変換
    (clamped * scale + 0.5) as i32
}

// 安全な逆数計算（ゼロ除算を防止）
fn rcp_safe(value: f32) -> f32 {
    if value != 0.0 {
        1.0 / value
    } else {
        0.0 // または適切なエラー処理
    }
}

pub fn normalize(val: f32, min: f32, max: f32) -> f32 {
    (val - min) / (max - min)
}

pub fn write_glb<W: Write>(
    writer: W,
    points: &PointCloud,
) -> Result<(Gltf, Vec<u8>), Box<dyn Error>> {
    let mut bin_content: Vec<u8> = Vec::new();
    let mut gltf_buffer_views = Vec::new();
    let mut gltf_accessors = Vec::new();

    const BYTE_STRIDE: usize = 2 * 6;
    // const BYTE_STRIDE: usize = 4 * 6;

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let mut raw_point_max = [f64::MIN; 3];
    let mut raw_point_min = [f64::MAX; 3];

    let scale = points.metadata.scale;
    let offset = points.metadata.offset;

    // TODO: color情報を書き込む方法

    for point in &points.points {
        // 元の座標にして利用する
        let raw_x = point.x as f32 * scale[0] as f32 + offset[0] as f32;
        let raw_y = point.y as f32 * scale[1] as f32 + offset[1] as f32;
        let raw_z = point.z as f32 * scale[2] as f32 + offset[2] as f32;

        raw_point_max[0] = raw_point_max[0].max(raw_x as f64);
        raw_point_max[1] = raw_point_max[1].max(raw_y as f64);
        raw_point_max[2] = raw_point_max[2].max(raw_z as f64);
        raw_point_min[0] = raw_point_min[0].min(raw_x as f64);
        raw_point_min[1] = raw_point_min[1].min(raw_y as f64);
        raw_point_min[2] = raw_point_min[2].min(raw_z as f64);

        // // scaleとoffsetを利用してエンコードした値を利用する
        // let x = point.x;
        // let y = point.y;
        // let z = point.z;

        // raw_point_max[0] = raw_point_max[0].max(x as f64);
        // raw_point_max[1] = raw_point_max[1].max(y as f64);
        // raw_point_max[2] = raw_point_max[2].max(z as f64);
        // raw_point_min[0] = raw_point_min[0].min(x as f64);
        // raw_point_min[1] = raw_point_min[1].min(y as f64);
        // raw_point_min[2] = raw_point_min[2].min(z as f64);

        // let r = point.color.r as u32;
        // let g = point.color.g as u32;
        // let b = point.color.b as u32;

        // LittleEndian::write_u32_into(
        //     &[
        //         (x as f32).to_bits(),
        //         (y as f32).to_bits(),
        //         (z as f32).to_bits(),
        //         r,
        //         g,
        //         b,
        //     ],
        //     &mut buffer,
        // );
        // bin_content.write_all(&buffer)?;
    }

    let mut position_max = [u16::MIN; 3];
    let mut position_min = [u16::MAX; 3];

    let pos_bits = 14;
    let pos_scale = &points.points.iter().fold(0f32, |result, point| {
        let raw_x = point.x as f32 * scale[0] as f32 + offset[0] as f32;
        let raw_y = point.y as f32 * scale[1] as f32 + offset[1] as f32;
        let raw_z = point.z as f32 * scale[2] as f32 + offset[2] as f32;

        result
            .max(raw_x - offset[0] as f32)
            .max(raw_y - offset[1] as f32)
            .max(raw_z - offset[2] as f32)
    });
    let pos_scale_inv = rcp_safe(*pos_scale);

    // quantize
    for point in &points.points {
        let raw_x = point.x as f32 * scale[0] as f32 + offset[0] as f32;
        let raw_y = point.y as f32 * scale[1] as f32 + offset[1] as f32;
        let raw_z = point.z as f32 * scale[2] as f32 + offset[2] as f32;
        // println!("raw_x: {}, raw_y: {}, raw_z: {}", raw_x, raw_y, raw_z);

        // let x = quantize(raw_x as f64, raw_point_min[0], raw_point_max[0]);
        // let y = quantize(raw_y as f64, raw_point_min[1], raw_point_max[1]);
        // let z = quantize(raw_z as f64, raw_point_min[2], raw_point_max[2]);
        // let norm_x = normalize(raw_x, raw_point_min[0] as f32, raw_point_max[0] as f32);
        // let norm_y = normalize(raw_y, raw_point_min[1] as f32, raw_point_max[1] as f32);
        // let norm_z = normalize(raw_z, raw_point_min[2] as f32, raw_point_max[2] as f32);
        // println!("norm_x: {}, norm_y: {}, norm_z: {}", norm_x, norm_y, norm_z);

        // let x = quantize_unorm(norm_x, pos_bits) as u16;
        // let y = quantize_unorm(norm_y, pos_bits) as u16;
        // let z = quantize_unorm(norm_z, pos_bits) as u16;
        let x = quantize_unorm((raw_x - offset[0] as f32) * pos_scale_inv, pos_bits) as u16;
        let y = quantize_unorm((raw_y - offset[1] as f32) * pos_scale_inv, pos_bits) as u16;
        let z = quantize_unorm((raw_z - offset[2] as f32) * pos_scale_inv, pos_bits) as u16;
        // let x = quantize_unorm(point.x as f32, pos_bits) as u16;
        // let y = quantize_unorm(point.y as f32, pos_bits) as u16;
        // let z = quantize_unorm(point.z as f32, pos_bits) as u16;

        // println!("quantize_x: {}, quantize_y: {}, quantize_z: {}", x, y, z);
        let r = point.color.r;
        let g = point.color.g;
        let b = point.color.b;

        position_max[0] = position_max[0].max(x);
        position_max[1] = position_max[1].max(y);
        position_max[2] = position_max[2].max(z);
        position_min[0] = position_min[0].min(x);
        position_min[1] = position_min[1].min(y);
        position_min[2] = position_min[2].min(z);
        // println!(
        // "position_max: {:?}, position_min: {:?}",
        // position_max, position_min
        // );

        LittleEndian::write_u16_into(&[x, y, z, r, g, b], &mut buffer);
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
        // component_type: ComponentType::Float,
        count: points.points.len() as u32,
        min: Some(position_min.iter().map(|&x| x as f64).collect()),
        max: Some(position_max.iter().map(|&x| x as f64).collect()),
        // min: Some(raw_point_min.to_vec()),
        // max: Some(raw_point_max.to_vec()),
        type_: AccessorType::Vec3,
        // normalized: true,
        ..Default::default()
    });

    gltf_accessors.push(Accessor {
        name: Some("colors".to_string()),
        buffer_view: Some(gltf_buffer_views.len() as u32 - 1),
        component_type: ComponentType::UnsignedShort,
        byte_offset: 2 * 3,
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
        // extensions: cesiumtiles_gltf_json::extensions::gltf::Gltf {
        //     ..Default::default()
        // }
        // .into(),
        extensions_used,
        extensions_required,
        ..Default::default()
    };

    cesiumtiles_gltf::glb::Glb {
        json: serde_json::to_vec(&gltf).unwrap().into(),
        bin: Some(bin_content.clone().into()),
    }
    .to_writer_with_alignment(writer, 8)?;

    Ok((gltf, bin_content))
}
