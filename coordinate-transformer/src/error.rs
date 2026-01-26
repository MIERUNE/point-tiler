#[derive(Debug)]
pub struct ProjError {
    pub code: i32,
    pub message: String,
    pub context: &'static str,
}

impl std::fmt::Display for ProjError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PROJ error ({}): {} {}",
            self.context, self.code, self.message
        )
    }
}

impl std::error::Error for ProjError {}
