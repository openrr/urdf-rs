use crate::common::*;
use serde::Deserialize;

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Inertial {
    #[serde(default)]
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum Geometry {
    Box {
        #[serde(with = "ss_vec3")]
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Capsule {
        radius: f64,
        length: f64,
    },
    Sphere {
        radius: f64,
    },
    Mesh {
        filename: String,
        #[serde(with = "ss_option_vec3", default)]
        scale: Option<[f64; 3]>,
    },
}

impl Default for Geometry {
    fn default() -> Geometry {
        Geometry::Box {
            size: [0.0f64, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Material {
    pub name: String,
    pub color: Option<ColorRGBA>,
    pub texture: Option<Texture>,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Visual {
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
    pub material: Option<Material>,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Collision {
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
}

/// Urdf Link element
/// See <http://wiki.ros.org/urdf/XML/link> for more detail.
#[derive(Debug, Deserialize, Clone)]
pub struct Link {
    pub name: String,
    #[serde(default)]
    pub inertial: Inertial,
    #[serde(default)]
    pub visual: Vec<Visual>,
    #[serde(default)]
    pub collision: Vec<Collision>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Axis {
    #[serde(with = "ss_vec3")]
    pub xyz: [f64; 3],
}

impl Default for Axis {
    fn default() -> Axis {
        Axis {
            xyz: [1.0f64, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct Pose {
    #[serde(with = "ss_vec3")]
    #[serde(default = "default_zero3")]
    pub xyz: [f64; 3],
    #[serde(with = "ss_vec3")]
    #[serde(default = "default_zero3")]
    pub rpy: [f64; 3],
}

fn default_zero3() -> [f64; 3] {
    [0.0f64, 0.0, 0.0]
}

impl Default for Pose {
    fn default() -> Pose {
        Pose {
            xyz: default_zero3(),
            rpy: default_zero3(),
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct LinkName {
    pub link: String,
}

#[derive(Debug, Deserialize, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct JointLimit {
    #[serde(default)]
    pub lower: f64,
    #[serde(default)]
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct SafetyController {
    #[serde(default)]
    pub soft_lower_limit: f64,
    #[serde(default)]
    pub soft_upper_limit: f64,
    #[serde(default)]
    pub k_position: f64,
    pub k_velocity: f64,
}

/// Urdf Joint element
/// See <http://wiki.ros.org/urdf/XML/joint> for more detail.
#[derive(Debug, Deserialize, Clone)]
pub struct Joint {
    pub name: String,
    #[serde(rename = "type")]
    pub joint_type: JointType,
    #[serde(default)]
    pub origin: Pose,
    pub parent: LinkName,
    pub child: LinkName,
    #[serde(default)]
    pub axis: Axis,
    #[serde(default)]
    pub limit: JointLimit,
    pub dynamics: Option<Dynamics>,
    pub mimic: Option<Mimic>,
    pub safety_controller: Option<SafetyController>,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Dynamics {
    #[serde(default)]
    pub damping: f64,
    #[serde(default)]
    pub friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, Deserialize, Clone)]
pub struct Robot {
    #[serde(default)]
    pub name: String,

    #[serde(rename = "link", default)]
    pub links: Vec<Link>,

    #[serde(rename = "joint", default)]
    pub joints: Vec<Joint>,

    #[serde(rename = "material", default)]
    pub materials: Vec<Material>,
}
