use std::{collections::HashMap, error::Error, io::Write};

use byteorder::{ByteOrder as _, LittleEndian};
use cesiumtiles_gltf_json::{
    Accessor, AccessorType, Buffer, BufferExtMeshoptCompression, BufferExtensions, BufferView,
    BufferViewTarget, ComponentType, Gltf, Mesh, MeshPrimitive, Node, Scene,
    extensions::{
        gltf::ext_structural_metadata as gltf_structural_metadata,
        mesh::{MeshPrimitive as MeshPrimitiveExtensions, ext_mesh_features},
    },
    extensions::buffer_view::{
        BufferViewExtensions, ExtMeshoptCompression, MeshoptCompressionFilter,
        MeshoptCompressionMode,
    },
};
use pcd_core::pointcloud::point::PointCloud;

/// Options for GLB generation
#[derive(Debug, Clone, Default)]
pub struct GlbOptions {
    pub quantize: bool,
    pub meshopt: bool,
    pub gzip_compress: bool,
}

pub fn quantize_unsigned_norm(value: f32, bits: i32) -> i32 {
    let max_value = (1i32 << bits) - 1i32;
    let scale = max_value as f32;

    let clamped = value.clamp(0.0, 1.0);

    (clamped * scale + 0.5) as i32
}

fn rcp(value: f32) -> f32 {
    if value != 0.0 { 1.0 / value } else { 0.0 }
}

/// Encode vertex buffer using meshopt with version 0 (EXT_meshopt_compression compatible).
fn encode_vertex_buffer_v0(
    vertices: &[u8],
    vertex_count: usize,
    vertex_size: usize,
) -> Result<Vec<u8>, Box<dyn Error>> {
    use meshopt::ffi;

    let bound = unsafe { ffi::meshopt_encodeVertexBufferBound(vertex_count, vertex_size) };
    let mut encoded = vec![0u8; bound];

    let encoded_size = unsafe {
        ffi::meshopt_encodeVertexBufferLevel(
            encoded.as_mut_ptr(),
            encoded.len(),
            vertices.as_ptr() as *const std::ffi::c_void,
            vertex_count,
            vertex_size,
            0, // level: 0 = fastest
            0, // version: 0 = EXT_meshopt_compression compatible
        )
    };

    if encoded_size == 0 && vertex_count > 0 {
        return Err("meshopt encode failed".into());
    }

    encoded.truncate(encoded_size);
    Ok(encoded)
}

/// Intermediate representation of built vertex buffer data.
struct VertexBufferInfo {
    /// Raw uncompressed vertex bytes
    bytes: Vec<u8>,
    /// Bytes per vertex
    byte_stride: usize,
    /// Number of vertices
    vertex_count: usize,
    /// Position accessor component type
    position_component_type: ComponentType,
    /// Whether position values are normalized
    position_normalized: bool,
    /// Position min values
    position_min: Vec<f64>,
    /// Position max values
    position_max: Vec<f64>,
    /// Color byte offset within vertex stride
    color_byte_offset: u32,
    /// Node translation
    translation: [f64; 3],
    /// Node scale (only for quantized positions)
    scale: Option<[f64; 3]>,
    /// Whether KHR_mesh_quantization extension is needed
    needs_quantization_extension: bool,
}

#[derive(Debug, Clone)]
struct MetadataProperty {
    name: &'static str,
    class_property: gltf_structural_metadata::ClassProperty,
    values: Vec<u8>,
    alignment: usize,
}

#[derive(Debug, Clone)]
struct StructuralMetadataInfo {
    feature_count: u32,
    properties: Vec<MetadataProperty>,
}

fn build_vertex_buffer_f32(points: &PointCloud) -> Result<VertexBufferInfo, Box<dyn Error>> {
    const BYTE_STRIDE: usize = (4 * 3) + (3 + 1); // 16

    let offset = points.metadata.offset;
    let vertex_count = points.points.len();

    let mut bytes: Vec<u8> = Vec::with_capacity(vertex_count * BYTE_STRIDE);
    let mut buffer = [0u8; BYTE_STRIDE];
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
        buffer[12..15].copy_from_slice(&[rgb8[0], rgb8[1], rgb8[2]]);
        buffer[15] = 0;

        bytes.write_all(&buffer)?;
    }

    Ok(VertexBufferInfo {
        bytes,
        byte_stride: BYTE_STRIDE,
        vertex_count,
        position_component_type: ComponentType::Float,
        position_normalized: false,
        position_min: min.to_vec(),
        position_max: max.to_vec(),
        color_byte_offset: 4 * 3,
        translation: offset,
        scale: None,
        needs_quantization_extension: false,
    })
}

fn build_vertex_buffer_quantized(points: &PointCloud) -> Result<VertexBufferInfo, Box<dyn Error>> {
    const BYTE_STRIDE: usize = (2 * 3 + 2) + (3 + 1); // 12

    let offset = points.metadata.offset;
    let vertex_count = points.points.len();

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

    let mut bytes: Vec<u8> = Vec::with_capacity(vertex_count * BYTE_STRIDE);
    let mut vertex_buf = [0u8; BYTE_STRIDE];

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

        quantized_position_max[0] = quantized_position_max[0].max(quantized_x);
        quantized_position_max[1] = quantized_position_max[1].max(quantized_y);
        quantized_position_max[2] = quantized_position_max[2].max(quantized_z);
        quantized_position_min[0] = quantized_position_min[0].min(quantized_x);
        quantized_position_min[1] = quantized_position_min[1].min(quantized_y);
        quantized_position_min[2] = quantized_position_min[2].min(quantized_z);

        LittleEndian::write_u16_into(
            &[quantized_x, quantized_y, quantized_z],
            &mut vertex_buf[0..6],
        );
        vertex_buf[6..8].copy_from_slice(&[0, 0]);
        vertex_buf[8..11].copy_from_slice(&[rgb8[0], rgb8[1], rgb8[2]]);
        vertex_buf[11] = 0;

        bytes.write_all(&vertex_buf)?;
    }

    Ok(VertexBufferInfo {
        bytes,
        byte_stride: BYTE_STRIDE,
        vertex_count,
        position_component_type: ComponentType::UnsignedShort,
        position_normalized: true,
        position_min: quantized_position_min.iter().map(|&x| x as f64).collect(),
        position_max: quantized_position_max.iter().map(|&x| x as f64).collect(),
        color_byte_offset: 2 * 3 + 2,
        translation: offset,
        scale: Some([
            common_scale as f64,
            common_scale as f64,
            common_scale as f64,
        ]),
        needs_quantization_extension: true,
    })
}

fn build_structural_metadata(points: &PointCloud) -> Option<StructuralMetadataInfo> {
    let feature_count = points.points.len() as u32;
    if feature_count == 0 {
        return None;
    }

    // Attribute presence is uniform across a file (driven by the CLI selection),
    // so the first point tells us which properties exist. This lets us skip the
    // accumulation loop entirely when nothing is selected, and avoid allocating
    // buffers for absent fields.
    let first = &points.points[0].attributes;
    let has_intensity = first.intensity.is_some();
    let has_return_number = first.return_number.is_some();
    let has_classification = first.classification.is_some();
    let has_scanner_channel = first.scanner_channel.is_some();
    let has_scan_angle = first.scan_angle.is_some();
    let has_user_data = first.user_data.is_some();
    let has_point_source_id = first.point_source_id.is_some();
    let has_gps_time = first.gps_time.is_some();

    if !(has_intensity
        || has_return_number
        || has_classification
        || has_scanner_channel
        || has_scan_angle
        || has_user_data
        || has_point_source_id
        || has_gps_time)
    {
        return None;
    }

    let n = points.points.len();
    let mut intensity = Vec::with_capacity(if has_intensity { n * 2 } else { 0 });
    let mut return_number = Vec::with_capacity(if has_return_number { n } else { 0 });
    let mut classification = Vec::with_capacity(if has_classification { n } else { 0 });
    let mut scanner_channel = Vec::with_capacity(if has_scanner_channel { n } else { 0 });
    let mut scan_angle = Vec::with_capacity(if has_scan_angle { n * 4 } else { 0 });
    let mut user_data = Vec::with_capacity(if has_user_data { n } else { 0 });
    let mut point_source_id = Vec::with_capacity(if has_point_source_id { n * 2 } else { 0 });
    let mut gps_time = Vec::with_capacity(if has_gps_time { n * 8 } else { 0 });

    for point in &points.points {
        let attrs = &point.attributes;

        if has_intensity {
            intensity.extend_from_slice(&attrs.intensity.unwrap_or_default().to_le_bytes());
        }
        if has_return_number {
            return_number.push(attrs.return_number.unwrap_or_default());
        }
        if has_classification {
            classification.push(attrs.classification.unwrap_or_default());
        }
        if has_scanner_channel {
            scanner_channel.push(attrs.scanner_channel.unwrap_or_default());
        }
        if has_scan_angle {
            scan_angle.extend_from_slice(&attrs.scan_angle.unwrap_or_default().to_le_bytes());
        }
        if has_user_data {
            user_data.push(attrs.user_data.unwrap_or_default());
        }
        if has_point_source_id {
            point_source_id
                .extend_from_slice(&attrs.point_source_id.unwrap_or_default().to_le_bytes());
        }
        if has_gps_time {
            gps_time.extend_from_slice(&attrs.gps_time.unwrap_or_default().to_le_bytes());
        }
    }

    let mut properties = Vec::new();
    if has_intensity {
        properties.push(MetadataProperty {
            name: "intensity",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint16),
                ..Default::default()
            },
            values: intensity,
            alignment: 2,
        });
    }
    if has_return_number {
        properties.push(MetadataProperty {
            name: "return_number",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint8),
                ..Default::default()
            },
            values: return_number,
            alignment: 1,
        });
    }
    if has_classification {
        properties.push(MetadataProperty {
            name: "classification",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint8),
                ..Default::default()
            },
            values: classification,
            alignment: 1,
        });
    }
    if has_scanner_channel {
        properties.push(MetadataProperty {
            name: "scanner_channel",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint8),
                ..Default::default()
            },
            values: scanner_channel,
            alignment: 1,
        });
    }
    if has_scan_angle {
        properties.push(MetadataProperty {
            name: "scan_angle",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Float32),
                ..Default::default()
            },
            values: scan_angle,
            alignment: 4,
        });
    }
    if has_user_data {
        properties.push(MetadataProperty {
            name: "user_data",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint8),
                ..Default::default()
            },
            values: user_data,
            alignment: 1,
        });
    }
    if has_point_source_id {
        properties.push(MetadataProperty {
            name: "point_source_id",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Uint16),
                ..Default::default()
            },
            values: point_source_id,
            alignment: 2,
        });
    }
    if has_gps_time {
        properties.push(MetadataProperty {
            name: "gps_time",
            class_property: gltf_structural_metadata::ClassProperty {
                type_: gltf_structural_metadata::ClassPropertyType::Scalar,
                component_type: Some(gltf_structural_metadata::ClassPropertyComponentType::Float64),
                ..Default::default()
            },
            values: gps_time,
            alignment: 8,
        });
    }

    if properties.is_empty() {
        None
    } else {
        Some(StructuralMetadataInfo {
            feature_count,
            properties,
        })
    }
}

fn append_aligned_bytes(target: &mut Vec<u8>, data: &[u8], alignment: usize) -> (u32, u32) {
    let pad = (alignment - (target.len() % alignment)) % alignment;
    if pad > 0 {
        target.extend(vec![0u8; pad]);
    }
    let offset = target.len() as u32;
    target.extend_from_slice(data);
    (offset, data.len() as u32)
}

/// Assemble a GLB from vertex buffer info, optionally applying meshopt compression.
///
/// When `meshopt` is true, uses a 2-buffer layout:
///   Buffer 0: compressed vertex data (in BIN chunk)
///   Buffer 1: fallback buffer (byteLength = uncompressed size, no data)
///   BufferView: points to Buffer 1, extension references Buffer 0
fn assemble_glb<'a>(
    info: VertexBufferInfo,
    meshopt: bool,
    metadata: Option<StructuralMetadataInfo>,
) -> Result<cesiumtiles_gltf::glb::Glb<'a>, Box<dyn Error>> {
    let uncompressed_len = info.bytes.len() as u32;

    let (mut gltf_buffers, mut gltf_buffer_views, mut bin_content) = if meshopt {
        let compressed = encode_vertex_buffer_v0(&info.bytes, info.vertex_count, info.byte_stride)?;
        let compressed_len = compressed.len() as u32;

        let buffers = vec![
            // Buffer 0: compressed data (in BIN chunk)
            Buffer {
                byte_length: compressed_len,
                ..Default::default()
            },
            // Buffer 1: fallback (no actual data)
            Buffer {
                byte_length: uncompressed_len,
                extensions: Some(BufferExtensions {
                    ext_meshopt_compression: Some(BufferExtMeshoptCompression { fallback: true }),
                    ..Default::default()
                }),
                ..Default::default()
            },
        ];

        let buffer_views = vec![BufferView {
            name: Some("vertices".to_string()),
            buffer: 1, // fallback buffer
            byte_offset: 0,
            byte_length: uncompressed_len,
            byte_stride: Some(info.byte_stride as u8),
            target: Some(BufferViewTarget::ArrayBuffer),
            extensions: Some(BufferViewExtensions {
                ext_meshopt_compression: Some(ExtMeshoptCompression {
                    buffer: 0,
                    byte_offset: 0,
                    byte_length: compressed_len,
                    byte_stride: info.byte_stride as u32,
                    count: info.vertex_count as u32,
                    mode: MeshoptCompressionMode::Attributes,
                    filter: MeshoptCompressionFilter::None,
                }),
                ..Default::default()
            }),
        }];

        (buffers, buffer_views, compressed)
    } else {
        let buffers = if uncompressed_len > 0 {
            vec![Buffer {
                byte_length: uncompressed_len,
                ..Default::default()
            }]
        } else {
            vec![]
        };

        let buffer_views = vec![BufferView {
            name: Some("vertices".to_string()),
            byte_offset: 0,
            byte_length: uncompressed_len,
            byte_stride: Some(info.byte_stride as u8),
            target: Some(BufferViewTarget::ArrayBuffer),
            ..Default::default()
        }];

        (buffers, buffer_views, info.bytes)
    };

    let mut root_extensions = None;

    if let Some(metadata) = metadata {
        let mut class = gltf_structural_metadata::Class::default();
        let mut property_table = gltf_structural_metadata::PropertyTable {
            class: "point".to_string(),
            count: metadata.feature_count,
            ..Default::default()
        };

        for property in metadata.properties {
            let (byte_offset, byte_length) =
                append_aligned_bytes(&mut bin_content, &property.values, property.alignment);

            gltf_buffer_views.push(BufferView {
                name: Some(format!("metadata_{}", property.name)),
                buffer: 0,
                byte_offset,
                byte_length,
                ..Default::default()
            });
            let values_buffer_view = (gltf_buffer_views.len() - 1) as u32;

            class
                .properties
                .insert(property.name.to_string(), property.class_property);
            property_table.properties.insert(
                property.name.to_string(),
                gltf_structural_metadata::PropertyTableProperty {
                    values: values_buffer_view,
                    ..Default::default()
                },
            );
        }

        gltf_buffers[0].byte_length = bin_content.len() as u32;

        root_extensions = Some(gltf_structural_metadata::ExtStructuralMetadata {
            schema: Some(gltf_structural_metadata::Schema {
                id: "point_tiler_attributes".to_string(),
                classes: HashMap::from([("point".to_string(), class)]),
                ..Default::default()
            }),
            property_tables: Some(vec![property_table]),
            ..Default::default()
        });
    }

    let gltf_accessors = vec![
        Accessor {
            name: Some("positions".to_string()),
            buffer_view: Some(0),
            component_type: info.position_component_type,
            count: info.vertex_count as u32,
            min: Some(info.position_min),
            max: Some(info.position_max),
            type_: AccessorType::Vec3,
            normalized: info.position_normalized,
            ..Default::default()
        },
        Accessor {
            name: Some("colors".to_string()),
            buffer_view: Some(0),
            component_type: ComponentType::UnsignedByte,
            byte_offset: info.color_byte_offset,
            count: info.vertex_count as u32,
            type_: AccessorType::Vec3,
            normalized: true,
            ..Default::default()
        },
    ];

    let primitive_extensions = root_extensions.as_ref().map(|_| MeshPrimitiveExtensions {
        ext_mesh_features: Some(ext_mesh_features::ExtMeshFeatures {
            feature_ids: vec![ext_mesh_features::FeatureId {
                feature_count: info.vertex_count as u32,
                property_table: Some(0),
                ..Default::default()
            }],
            ..Default::default()
        }),
        ..Default::default()
    });

    let gltf_meshes = vec![Mesh {
        primitives: vec![MeshPrimitive {
            attributes: HashMap::from_iter(vec![
                ("POSITION".to_string(), 0),
                ("COLOR_0".to_string(), 1),
            ]),
            mode: cesiumtiles_gltf_json::PrimitiveMode::Points,
            extensions: primitive_extensions,
            ..Default::default()
        }],
        ..Default::default()
    }];

    let mut extensions_used = Vec::new();
    let mut extensions_required = Vec::new();
    if info.needs_quantization_extension {
        extensions_used.push("KHR_mesh_quantization".to_string());
        extensions_required.push("KHR_mesh_quantization".to_string());
    }
    if meshopt {
        extensions_used.push("EXT_meshopt_compression".to_string());
        extensions_required.push("EXT_meshopt_compression".to_string());
    }
    if root_extensions.is_some() {
        extensions_used.push("EXT_structural_metadata".to_string());
        extensions_required.push("EXT_structural_metadata".to_string());
        extensions_used.push("EXT_mesh_features".to_string());
        extensions_required.push("EXT_mesh_features".to_string());
    }

    let mut node = Node {
        mesh: Some(0),
        translation: info.translation,
        ..Default::default()
    };
    if let Some(scale) = info.scale {
        node.scale = scale;
    }

    let gltf = Gltf {
        scenes: vec![Scene {
            nodes: Some(vec![0]),
            ..Default::default()
        }],
        nodes: vec![node],
        meshes: gltf_meshes,
        accessors: gltf_accessors,
        buffer_views: gltf_buffer_views,
        buffers: gltf_buffers,
        extensions_used,
        extensions_required,
        extensions: root_extensions.map(|ext| cesiumtiles_gltf_json::extensions::gltf::Gltf {
            ext_structural_metadata: Some(ext),
            others: HashMap::new(),
        }),
        ..Default::default()
    };

    Ok(cesiumtiles_gltf::glb::Glb {
        json: serde_json::to_vec(&gltf).unwrap().into(),
        bin: Some(bin_content.into()),
    })
}

pub fn generate_glb_with_options<'a>(
    points: PointCloud,
    options: &GlbOptions,
) -> Result<cesiumtiles_gltf::glb::Glb<'a>, Box<dyn Error>> {
    let metadata = build_structural_metadata(&points);
    let info = if options.quantize {
        build_vertex_buffer_quantized(&points)?
    } else {
        build_vertex_buffer_f32(&points)?
    };
    assemble_glb(info, options.meshopt, metadata)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pcd_core::pointcloud::point::{Color, Metadata, Point, PointAttributes};

    fn make_test_points() -> PointCloud {
        PointCloud {
            points: vec![
                Point {
                    x: 10.0,
                    y: 20.0,
                    z: 30.0,
                    color: Color {
                        r: 65535,
                        g: 0,
                        b: 0,
                    },
                    attributes: PointAttributes {
                        intensity: None,
                        return_number: None,
                        classification: None,
                        scanner_channel: None,
                        scan_angle: None,
                        user_data: None,
                        point_source_id: None,
                        gps_time: None,
                    },
                },
                Point {
                    x: 11.0,
                    y: 21.0,
                    z: 31.0,
                    color: Color {
                        r: 0,
                        g: 65535,
                        b: 0,
                    },
                    attributes: PointAttributes {
                        intensity: None,
                        return_number: None,
                        classification: None,
                        scanner_channel: None,
                        scan_angle: None,
                        user_data: None,
                        point_source_id: None,
                        gps_time: None,
                    },
                },
                Point {
                    x: 12.0,
                    y: 22.0,
                    z: 32.0,
                    color: Color {
                        r: 0,
                        g: 0,
                        b: 65535,
                    },
                    attributes: PointAttributes {
                        intensity: None,
                        return_number: None,
                        classification: None,
                        scanner_channel: None,
                        scan_angle: None,
                        user_data: None,
                        point_source_id: None,
                        gps_time: None,
                    },
                },
            ],
            metadata: Metadata {
                point_count: 3,
                offset: [10.0, 20.0, 30.0],
                ..Default::default()
            },
        }
    }

    fn make_test_points_with_attributes() -> PointCloud {
        PointCloud {
            points: vec![
                Point {
                    x: 10.0,
                    y: 20.0,
                    z: 30.0,
                    color: Color {
                        r: 65535,
                        g: 0,
                        b: 0,
                    },
                    attributes: PointAttributes {
                        intensity: Some(1000),
                        return_number: Some(1),
                        classification: Some(2),
                        scanner_channel: Some(0),
                        scan_angle: Some(1.5),
                        user_data: Some(7),
                        point_source_id: Some(42),
                        gps_time: Some(10.25),
                    },
                },
                Point {
                    x: 11.0,
                    y: 21.0,
                    z: 31.0,
                    color: Color {
                        r: 0,
                        g: 65535,
                        b: 0,
                    },
                    attributes: PointAttributes {
                        intensity: Some(2000),
                        return_number: Some(2),
                        classification: Some(6),
                        scanner_channel: Some(1),
                        scan_angle: Some(-3.0),
                        user_data: Some(8),
                        point_source_id: Some(43),
                        gps_time: Some(11.5),
                    },
                },
            ],
            metadata: Metadata {
                point_count: 2,
                offset: [10.0, 20.0, 30.0],
                ..Default::default()
            },
        }
    }

    fn parse_glb_json(glb: &cesiumtiles_gltf::glb::Glb) -> serde_json::Value {
        serde_json::from_slice(&glb.json).unwrap()
    }

    #[test]
    fn test_generate_glb_basic() {
        let points = make_test_points();
        let glb = generate_glb_with_options(points, &GlbOptions::default()).unwrap();
        let json = parse_glb_json(&glb);

        // Verify buffer structure: single buffer
        let buffers = json["buffers"].as_array().unwrap();
        assert_eq!(buffers.len(), 1);

        // Verify buffer view
        let buffer_views = json["bufferViews"].as_array().unwrap();
        assert_eq!(buffer_views.len(), 1);
        assert_eq!(buffer_views[0]["buffer"], 0);
        assert_eq!(buffer_views[0]["byteStride"], 16);

        // Verify accessors
        let accessors = json["accessors"].as_array().unwrap();
        assert_eq!(accessors.len(), 2);
        // Position: Float
        assert_eq!(accessors[0]["componentType"], 5126);
        assert_eq!(accessors[0]["type"], "VEC3");
        assert_eq!(accessors[0]["count"], 3);
        // Color: UnsignedByte, normalized
        assert_eq!(accessors[1]["componentType"], 5121);
        assert_eq!(accessors[1]["byteOffset"], 12);
        assert!(accessors[1]["normalized"].as_bool().unwrap());

        // No extensions (empty arrays are omitted from JSON)
        let ext_used = json.get("extensionsUsed").and_then(|v| v.as_array());
        assert!(ext_used.is_none() || ext_used.unwrap().is_empty());
        let ext_req = json.get("extensionsRequired").and_then(|v| v.as_array());
        assert!(ext_req.is_none() || ext_req.unwrap().is_empty());

        // BIN chunk should exist
        assert!(glb.bin.is_some());
        assert_eq!(glb.bin.as_ref().unwrap().len(), 3 * 16);
    }

    #[test]
    fn test_generate_quantized_glb() {
        let points = make_test_points();
        let glb = generate_glb_with_options(
            points,
            &GlbOptions {
                quantize: true,
                ..Default::default()
            },
        )
        .unwrap();
        let json = parse_glb_json(&glb);

        // Single buffer
        let buffers = json["buffers"].as_array().unwrap();
        assert_eq!(buffers.len(), 1);

        // Buffer view: stride 12
        let buffer_views = json["bufferViews"].as_array().unwrap();
        assert_eq!(buffer_views[0]["byteStride"], 12);

        // Position: UnsignedShort, normalized
        let accessors = json["accessors"].as_array().unwrap();
        assert_eq!(accessors[0]["componentType"], 5123);
        assert!(accessors[0]["normalized"].as_bool().unwrap());
        // Color: byte offset = 8
        assert_eq!(accessors[1]["byteOffset"], 8);

        // KHR_mesh_quantization extension
        let ext_used: Vec<&str> = json["extensionsUsed"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert!(ext_used.contains(&"KHR_mesh_quantization"));
        assert!(!ext_used.contains(&"EXT_meshopt_compression"));

        // Node should have scale
        let nodes = json["nodes"].as_array().unwrap();
        assert!(nodes[0].get("scale").is_some());

        // BIN: 3 vertices * 12 bytes
        assert_eq!(glb.bin.as_ref().unwrap().len(), 3 * 12);
    }

    #[test]
    fn test_generate_glb_with_structural_metadata() {
        let points = make_test_points_with_attributes();
        let glb = generate_glb_with_options(points, &GlbOptions::default()).unwrap();
        let json = parse_glb_json(&glb);

        let ext_used: Vec<&str> = json["extensionsUsed"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert!(ext_used.contains(&"EXT_structural_metadata"));
        assert!(ext_used.contains(&"EXT_mesh_features"));

        let schema_props = &json["extensions"]["EXT_structural_metadata"]["schema"]["classes"]["point"]
            ["properties"];
        for key in [
            "intensity",
            "return_number",
            "classification",
            "scanner_channel",
            "scan_angle",
            "user_data",
            "point_source_id",
            "gps_time",
        ] {
            assert!(schema_props.get(key).is_some(), "missing schema property: {key}");
        }

        let mesh_ext = &json["meshes"][0]["primitives"][0]["extensions"]["EXT_mesh_features"];
        assert_eq!(mesh_ext["featureIds"][0]["featureCount"], 2);
        assert_eq!(mesh_ext["featureIds"][0]["propertyTable"], 0);

        let property_table = &json["extensions"]["EXT_structural_metadata"]["propertyTables"][0];
        assert_eq!(property_table["count"], 2);
        let property_table_props = property_table["properties"].as_object().unwrap();
        assert_eq!(property_table_props.len(), 8);
    }

    #[test]
    fn test_generate_meshopt_glb() {
        let points = make_test_points();
        let glb = generate_glb_with_options(
            points,
            &GlbOptions {
                meshopt: true,
                ..Default::default()
            },
        )
        .unwrap();
        let json = parse_glb_json(&glb);

        // 2-buffer layout
        let buffers = json["buffers"].as_array().unwrap();
        assert_eq!(buffers.len(), 2);
        // Buffer 1 should have fallback extension
        assert!(
            buffers[1]["extensions"]["EXT_meshopt_compression"]["fallback"]
                .as_bool()
                .unwrap()
        );

        // Buffer view should reference buffer 1 with meshopt extension
        let buffer_views = json["bufferViews"].as_array().unwrap();
        assert_eq!(buffer_views[0]["buffer"], 1);
        let meshopt_ext = &buffer_views[0]["extensions"]["EXT_meshopt_compression"];
        assert_eq!(meshopt_ext["buffer"], 0);
        assert_eq!(meshopt_ext["byteStride"], 16);
        assert_eq!(meshopt_ext["count"], 3);
        assert_eq!(meshopt_ext["mode"], "ATTRIBUTES");

        // Position: Float (not quantized)
        let accessors = json["accessors"].as_array().unwrap();
        assert_eq!(accessors[0]["componentType"], 5126);

        // EXT_meshopt_compression extension only
        let ext_used: Vec<&str> = json["extensionsUsed"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert!(ext_used.contains(&"EXT_meshopt_compression"));
        assert!(!ext_used.contains(&"KHR_mesh_quantization"));

        // Node should NOT have scale
        let nodes = json["nodes"].as_array().unwrap();
        assert!(nodes[0].get("scale").is_none());

        // BIN is compressed, so smaller than uncompressed
        let bin_len = glb.bin.as_ref().unwrap().len();
        assert!(bin_len > 0);
    }

    #[test]
    fn test_generate_quantized_meshopt_glb() {
        let points = make_test_points();
        let glb = generate_glb_with_options(
            points,
            &GlbOptions {
                quantize: true,
                meshopt: true,
                ..Default::default()
            },
        )
        .unwrap();
        let json = parse_glb_json(&glb);

        // 2-buffer layout
        let buffers = json["buffers"].as_array().unwrap();
        assert_eq!(buffers.len(), 2);

        // Buffer view: stride 12, meshopt extension
        let buffer_views = json["bufferViews"].as_array().unwrap();
        let meshopt_ext = &buffer_views[0]["extensions"]["EXT_meshopt_compression"];
        assert_eq!(meshopt_ext["byteStride"], 12);

        // Position: UnsignedShort, normalized
        let accessors = json["accessors"].as_array().unwrap();
        assert_eq!(accessors[0]["componentType"], 5123);
        assert!(accessors[0]["normalized"].as_bool().unwrap());

        // Both extensions
        let ext_used: Vec<&str> = json["extensionsUsed"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert!(ext_used.contains(&"KHR_mesh_quantization"));
        assert!(ext_used.contains(&"EXT_meshopt_compression"));

        // Node should have scale
        let nodes = json["nodes"].as_array().unwrap();
        assert!(nodes[0].get("scale").is_some());
    }

    #[test]
    fn test_glb_roundtrip_parseable() {
        // Verify all 4 variants produce parseable GLBs
        for (quantize, meshopt) in [(false, false), (true, false), (false, true), (true, true)] {
            let points = make_test_points();
            let options = GlbOptions {
                quantize,
                meshopt,
                ..Default::default()
            };
            let glb = generate_glb_with_options(points, &options).unwrap();

            // Write to bytes and parse back
            let mut buf = Vec::new();
            glb.to_writer_with_alignment(&mut buf, 8).unwrap();
            let parsed = cesiumtiles_gltf::glb::Glb::from_reader(&buf[..]).unwrap();

            // Verify JSON is valid
            let _json: serde_json::Value = serde_json::from_slice(&parsed.json).unwrap();
            assert!(parsed.bin.is_some());
        }
    }
}
