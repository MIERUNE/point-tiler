use std::{error::Error, ffi::OsStr, path::PathBuf};

use las::LasParserProvider;
use pcd_core::pointcloud::point::PointCloud;
use projection_transform::crs::EpsgCode;

pub mod csv;
pub mod las;

pub trait ParserProvider {
    fn get_parser(&self) -> Box<dyn Parser>;
}

pub trait Parser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>>;
}

pub enum Extension {
    Las,
    Laz,
    Csv,
    Txt,
}

pub fn get_extension(extension: &str) -> Extension {
    match extension {
        "las" => Extension::Las,
        "laz" => Extension::Laz,
        "csv" => Extension::Csv,
        "txt" => Extension::Txt,
        _ => panic!("Unsupported extension"),
    }
}
