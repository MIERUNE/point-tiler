#[derive(Debug)]
pub struct ProjError {
    pub message: String,
    pub context: &'static str,
}

impl ProjError {
    pub(crate) fn new(context: &'static str, err: impl std::fmt::Display) -> Self {
        Self {
            message: err.to_string(),
            context,
        }
    }
}

impl std::fmt::Display for ProjError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PROJ error ({}): {}", self.context, self.message)
    }
}

impl std::error::Error for ProjError {}
