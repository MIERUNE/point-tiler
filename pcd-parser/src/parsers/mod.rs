use std::error::Error;

use crate::pointcloud::point::PointCloud;

pub mod csv;
pub mod las;

pub trait ParserProvider {
    fn get_parser(&self) -> Box<dyn Parser>;
}

pub trait Parser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>>;
}
