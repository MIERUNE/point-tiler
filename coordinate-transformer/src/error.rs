use std::fmt;

use proj_sys_transformer::ProjError;

#[derive(Debug)]
pub enum ProjectionError {
    Proj(ProjError),
}

impl fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Proj(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ProjectionError {}

impl From<ProjError> for ProjectionError {
    fn from(value: ProjError) -> Self {
        Self::Proj(value)
    }
}
