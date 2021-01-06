use serde_xml_rs;
use std;
use std::error::Error;
use std::fmt;
use std::string;
use xml;

#[derive(Debug)]
pub enum UrdfError {
    File(::std::io::Error),
    Xml(::serde_xml_rs::Error),
    RustyXml(::xml::BuilderError),
    Parse(String),
    Command(String),
}

pub type Result<T> = ::std::result::Result<T, UrdfError>;

impl fmt::Display for UrdfError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            UrdfError::File(ref err) => err.fmt(f),
            UrdfError::Xml(ref err) => err.fmt(f),
            UrdfError::RustyXml(ref err) => err.fmt(f),
            UrdfError::Parse(ref msg) => write!(f, "parse error {}", msg),
            UrdfError::Command(ref msg) => write!(f, "command error {}", msg),
        }
    }
}

impl Error for UrdfError {}

impl From<std::io::Error> for UrdfError {
    fn from(err: std::io::Error) -> UrdfError {
        UrdfError::File(err)
    }
}

impl From<serde_xml_rs::Error> for UrdfError {
    fn from(err: serde_xml_rs::Error) -> UrdfError {
        UrdfError::Xml(err)
    }
}

impl From<xml::BuilderError> for UrdfError {
    fn from(err: xml::BuilderError) -> UrdfError {
        UrdfError::RustyXml(err)
    }
}

impl<'a> From<&'a str> for UrdfError {
    fn from(err: &'a str) -> UrdfError {
        UrdfError::Command(err.to_owned())
    }
}

impl From<string::FromUtf8Error> for UrdfError {
    fn from(err: string::FromUtf8Error) -> UrdfError {
        UrdfError::Command(err.to_string())
    }
}
