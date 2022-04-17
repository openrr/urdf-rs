use crate::common::space_seperated_vectors::*;
use serde::Deserialize;

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Mass {
    pub value: f64,
}

#[derive(Deserialize, Debug, Clone)]
pub struct Vec3 {
    #[serde(with = "ss_vec3")]
    pub data: [f64; 3],
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct ColorRGBA {
    #[serde(with = "ss_vec4")]
    pub rgba: [f64; 4],
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct ColorRGB {
    #[serde(with = "ss_vec3")]
    pub rgba: [f64; 3],
}

#[derive(Debug, Deserialize, Clone)]
pub struct Texture {
    pub filename: String,
}

impl Default for Texture {
    fn default() -> Texture {
        Texture {
            filename: "".to_string(),
        }
    }
}
