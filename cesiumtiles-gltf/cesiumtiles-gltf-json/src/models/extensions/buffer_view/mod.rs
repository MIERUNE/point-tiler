use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Compression mode for EXT_meshopt_compression
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MeshoptCompressionMode {
    #[serde(rename = "ATTRIBUTES")]
    Attributes,
    #[serde(rename = "TRIANGLES")]
    Triangles,
    #[serde(rename = "INDICES")]
    Indices,
}

/// Post-decompression filter for EXT_meshopt_compression
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Default)]
pub enum MeshoptCompressionFilter {
    #[default]
    #[serde(rename = "NONE")]
    None,
    #[serde(rename = "OCTAHEDRAL")]
    Octahedral,
    #[serde(rename = "QUATERNION")]
    Quaternion,
    #[serde(rename = "EXPONENTIAL")]
    Exponential,
}

/// The EXT_meshopt_compression extension for a bufferView
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ExtMeshoptCompression {
    pub buffer: u32,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub byte_offset: u32,
    pub byte_length: u32,
    pub byte_stride: u32,
    pub count: u32,
    pub mode: MeshoptCompressionMode,
    #[serde(default, skip_serializing_if = "is_default_filter")]
    pub filter: MeshoptCompressionFilter,
}

fn is_zero(v: &u32) -> bool {
    *v == 0
}

fn is_default_filter(f: &MeshoptCompressionFilter) -> bool {
    *f == MeshoptCompressionFilter::None
}

/// Extensions container for BufferView
#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Clone)]
pub struct BufferViewExtensions {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "EXT_meshopt_compression")]
    pub ext_meshopt_compression: Option<ExtMeshoptCompression>,

    #[serde(flatten)]
    pub others: HashMap<String, Value>,
}
