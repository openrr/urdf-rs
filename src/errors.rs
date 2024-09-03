use thiserror::Error;

/// Alias for a `Result` with the error type `UrdfError`.
pub type Result<T> = std::result::Result<T, UrdfError>;

#[derive(Debug, Error)]
#[error(transparent)]
pub struct UrdfError(#[from] ErrorKind);

// Hiding error variants from a library's public error type to prevent
// dependency updates from becoming breaking changes.
// We can add `UrdfErrorKind` enum or `is_*` methods that indicate the kind of
// error if needed, but don't expose dependencies' types directly in the
// public API.
#[derive(Debug, Error)]
pub(crate) enum ErrorKind {
    #[error(transparent)]
    File(#[from] std::io::Error),
    #[error(transparent)]
    Xml(#[from] serde_xml_rs::Error),
    #[error(transparent)]
    RustyXml(#[from] xml::BuilderError),
    #[error(transparent)]
    QuickXml(#[from] quick_xml::DeError),
    #[error("command error {}\n--- stdout\n{}\n--- stderr\n{}", .msg, .stdout, .stderr)]
    Command {
        msg: String,
        stdout: String,
        stderr: String,
    },
    #[error("{}", .0)]
    Other(String),
}

impl UrdfError {
    pub(crate) fn new(err: impl Into<ErrorKind>) -> Self {
        Self(err.into())
    }
}

impl From<std::io::Error> for UrdfError {
    fn from(err: std::io::Error) -> UrdfError {
        ErrorKind::File(err).into()
    }
}

impl From<&str> for UrdfError {
    fn from(err: &str) -> UrdfError {
        ErrorKind::Other(err.to_owned()).into()
    }
}

impl From<String> for UrdfError {
    fn from(err: String) -> UrdfError {
        ErrorKind::Other(err).into()
    }
}

impl From<std::string::FromUtf8Error> for UrdfError {
    fn from(err: std::string::FromUtf8Error) -> UrdfError {
        ErrorKind::Other(err.to_string()).into()
    }
}

// Note: These implementations are intentionally not-exist to prevent dependency
// updates from becoming breaking changes.
// impl From<serde_xml_rs::Error> for UrdfError
// impl From<xml::BuilderError> for UrdfError
