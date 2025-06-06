use std::error::Error;

use pcd_core::pointcloud::point::PointCloud;

pub mod csv;
pub mod las;

pub trait ParserProvider {
    fn get_parser(&self) -> Box<dyn Parser>;
}

pub trait Parser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>>;
}

#[derive(Debug, Clone, Copy)]
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
