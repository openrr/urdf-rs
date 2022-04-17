// use crate::common::*;
use serde::Deserialize;

// #[derive(Debug, Deserialize, Clone)]
// #[serde(rename_all = "snake_case")]
// pub enum Geometry {
//     Empty,
//     Box {
//         #[serde(with = "ss_vec3")]
//         size: [f64; 3],
//     },
//     Capsule {
//         radius: f64,
//         length: f64,
//     },
//     Cylinder {
//         radius: f64,
//         length: f64,
//     },
//     // Ellipsoid,
//     // Heightmap,
//     // Image,
//     Mesh {
//         filename: String,
//         #[serde(with = "crate::common::ss_option_vec3", default)]
//         scale: Option<[f64; 3]>,
//     },
//     // Plane,
//     // Polyline,
//     Sphere {
//         radius: f64,
//     },
// }

// impl Default for Geometry {
//     fn default() -> Geometry {
//         Geometry::Box {
//             size: [0.0f64, 0.0, 0.0],
//         }
//     }
// }

// #[derive(Debug, Deserialize, Default, Clone)]
// pub struct Material {
//     pub name: String,
//     pub color: Option<ColorRGBA>,
//     pub texture: Option<Texture>,
// }

// #[derive(Debug, Deserialize, Default, Clone)]
// pub struct Visual {
//     pub name: Option<String>,
//     #[serde(default)]
//     pub origin: Pose,
//     pub geometry: Geometry,
//     pub material: Option<Material>,
// }

// /// Urdf Link element
// /// See <http://wiki.ros.org/urdf/XML/link> for more detail.
// #[derive(Debug, Deserialize, Clone)]
// pub struct Link {
//     pub name: String,
//     #[serde(default)]
//     pub pose: Pose,
//     #[serde(default)]
//     pub visual: Vec<Visual>,
// }

// #[derive(Debug, Deserialize, Clone)]
// pub struct Axis {
//     #[serde(with = "crate::common::ss_vec3")]
//     pub xyz: [f64; 3],
// }

// impl Default for Axis {
//     fn default() -> Axis {
//         Axis {
//             xyz: [0.0f64, 0.0, 1.0],
//         }
//     }
// }

// #[derive(Debug, Deserialize, Clone)]
// pub struct Pose {
//     #[serde(with = "crate::common::ss_vec3")]
//     #[serde(default = "default_zero3")]
//     pub xyz: [f64; 3],
//     #[serde(with = "crate::common::ss_vec3")]
//     #[serde(default = "default_zero3")]
//     pub rpy: [f64; 3],
// }

// fn default_zero3() -> [f64; 3] {
//     [0.0f64, 0.0, 0.0]
// }

// impl Default for Pose {
//     fn default() -> Pose {
//         Pose {
//             xyz: default_zero3(),
//             rpy: default_zero3(),
//         }
//     }
// }

// #[derive(Debug, Deserialize, Clone)]
// pub struct LinkName {
//     pub link: String,
// }

// #[derive(Debug, Deserialize, Clone, PartialEq, Eq)]
// #[serde(rename_all = "snake_case")]
// pub enum JointType {
//     Revolute,
//     Continuous,
//     Prismatic,
//     Fixed,
//     Floating,
//     Planar,
//     Spherical,
// }

// #[derive(Debug, Deserialize, Default, Clone)]
// pub struct JointLimit {
//     #[serde(default)]
//     pub lower: f64,
//     #[serde(default)]
//     pub upper: f64,
//     pub effort: f64,
//     pub velocity: f64,
// }

// /// Urdf Joint element
// /// See <http://wiki.ros.org/urdf/XML/joint> for more detail.
// #[derive(Debug, Deserialize, Clone)]
// pub struct Joint {
//     pub name: String,
//     #[serde(rename = "type")]
//     pub joint_type: JointType,
//     #[serde(default)]
//     pub origin: Pose,
//     pub parent: LinkName,
//     pub child: LinkName,
//     #[serde(default)]
//     pub axis: Axis,
//     #[serde(default)]
//     pub limit: JointLimit,
// }

/// Top level struct representing an SDFormat <model> element.
/// http://sdformat.org/spec?ver=1.9&elem=model
#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct Model {
    #[serde(default)]
    pub name: String,
    // #[serde(rename = "link", default)]
    // pub links: Vec<Link>,

    // #[serde(rename = "joint", default)]
    // pub joints: Vec<Joint>,

    // #[serde(rename = "material", default)]
    // pub materials: Vec<Material>,
}

/// Top level struct representing an SDFormat <world> element.
/// http://sdformat.org/spec?ver=1.9&elem=world
#[derive(Debug, Deserialize, Clone, PartialEq)]
#[serde(rename = "world")]
pub struct World {
    #[serde(default)]
    pub name: String,
    // #[serde(rename = "frame", default)]
    // pub frames: Vec<Frame>,
    #[serde(rename = "model", default)]
    pub models: Vec<Model>,
}

impl World {
    pub fn models(&self) -> Vec<&Model> {
        self.models.iter().collect()
    }
}

/// Top level enum to access root element of a parsed sdf.
/// http://sdformat.org/spec?ver=1.9&elem=sdf
#[derive(Debug, Clone, PartialEq)]
pub enum Sdf {
    Model(Model),
    Worlds(Vec<World>),
    // TODO: Actor
    // TODO: Light
}

impl Sdf {
    pub fn models(&self) -> Vec<&Model> {
        match &self {
            Sdf::Model(m) => {
                vec![&m]
            },
            Sdf::Worlds(ws) => ws.iter().map(|w| w.models()).flatten().collect(),
        }
    }
}

#[derive(Debug, Deserialize, Clone, PartialEq)]
#[serde(rename = "sdf")]
pub struct SDFormat {
    #[serde(rename = "world", default)]
    pub worlds: Vec<World>,
    #[serde(rename = "model", default)]
    pub models: Vec<Model>,
}
