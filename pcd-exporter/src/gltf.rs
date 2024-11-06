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

    // const VERTEX_BYTE_STRIDE: usize = 4 * 6; // 4byte(i32) x 3 (x, y, z)
    // const COLOR_BYTE_STRIDE: usize = 2 * 4; // 2byte(u16) x 3 (r, g, b)
    const BYTE_STRIDE: usize = 8 * 6;

    let buffer_offset = bin_content.len();
    let mut buffer = [0u8; BYTE_STRIDE];

    let mut position_max = [f64::MIN; 3];
    let mut position_min = [f64::MAX; 3];

    for point in &points.points {
        let x = point.x as f64;
        let y = point.y as f64;
        let z = point.z as f64;
        let r = point.color.r as f64;
        let g = point.color.g as f64;
        let b = point.color.b as f64;

        position_max[0] = position_max[0].max(x);
        position_max[1] = position_max[1].max(y);
        position_max[2] = position_max[2].max(z);

        position_min[0] = position_min[0].min(x);
        position_min[1] = position_min[1].min(y);
        position_min[2] = position_min[2].min(z);

        LittleEndian::write_f64_into(&[x, y, z, r, g, b], &mut buffer);
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
        component_type: ComponentType::Float,
        byte_offset: 8 * 3,
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
