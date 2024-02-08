use serde::{Deserialize, Serialize};

use std::ops::{Deref, DerefMut};

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mass {
    #[serde(rename = "@value")]
    pub value: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Inertia {
    #[serde(rename = "@ixx")]
    pub ixx: f64,
    #[serde(rename = "@ixy")]
    pub ixy: f64,
    #[serde(rename = "@ixz")]
    pub ixz: f64,
    #[serde(rename = "@iyy")]
    pub iyy: f64,
    #[serde(rename = "@iyz")]
    pub iyz: f64,
    #[serde(rename = "@izz")]
    pub izz: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Inertial {
    #[serde(default)]
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum Geometry {
    Box {
        #[serde(rename = "@size")]
        size: Vec3,
    },
    Cylinder {
        #[serde(rename = "@radius")]
        radius: f64,
        #[serde(rename = "@length")]
        length: f64,
    },
    Capsule {
        #[serde(rename = "@radius")]
        radius: f64,
        #[serde(rename = "@length")]
        length: f64,
    },
    Sphere {
        #[serde(rename = "@radius")]
        radius: f64,
    },
    Mesh {
        #[serde(rename = "@filename")]
        filename: String,
        #[serde(rename = "@scale", skip_serializing_if = "Option::is_none")]
        scale: Option<Vec3>,
    },
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct GeometryTag {
    #[serde(rename = "$value")]
    pub value: Geometry,
}

impl From<Geometry> for GeometryTag {
    fn from(geom: Geometry) -> Self {
        Self { value: geom }
    }
}

impl Deref for GeometryTag {
    type Target = Geometry;

    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl DerefMut for GeometryTag {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Color {
    #[serde(rename = "@rgba")]
    pub rgba: Vec4,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Texture {
    #[serde(rename = "@filename")]
    pub filename: String,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Material {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<Color>,
    #[serde(rename = "@texture", skip_serializing_if = "Option::is_none")]
    pub texture: Option<Texture>,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Visual {
    #[serde(rename = "@name", skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: GeometryTag,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material: Option<Material>,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Collision {
    #[serde(rename = "@name", skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: GeometryTag,
}

/// Urdf Link element
/// See <http://wiki.ros.org/urdf/XML/link> for more detail.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Link {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(default)]
    pub inertial: Inertial,
    #[serde(default)]
    pub visual: Vec<Visual>,
    #[serde(default)]
    pub collision: Vec<Collision>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone, Copy, PartialEq)]
pub struct Vec3(#[serde(rename = "$text")] pub [f64; 3]);

impl Deref for Vec3 {
    type Target = [f64; 3];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Vec3 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy, PartialEq)]
pub struct Vec4(#[serde(rename = "$text")] pub [f64; 4]);

impl Deref for Vec4 {
    type Target = [f64; 4];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Vec4 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Axis {
    #[serde(rename = "@xyz")]
    pub xyz: Vec3,
}

impl Default for Axis {
    fn default() -> Axis {
        Axis {
            xyz: Vec3([1.0, 0.0, 0.0]),
        }
    }
}

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub struct Pose {
    #[serde(rename = "@xyz", default)]
    pub xyz: Vec3,
    #[serde(rename = "@rpy", default)]
    pub rpy: Vec3,
}

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub struct LinkName {
    #[serde(rename = "@link")]
    pub link: String,
}

#[derive(Debug, Deserialize, Serialize, Clone, PartialEq, Eq)]
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

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct JointLimit {
    #[serde(rename = "@lower", default)]
    pub lower: f64,
    #[serde(rename = "@upper", default)]
    pub upper: f64,
    #[serde(rename = "@effort")]
    pub effort: f64,
    #[serde(rename = "@velocity")]
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mimic {
    #[serde(rename = "@joint")]
    pub joint: String,
    #[serde(rename = "@multiplier", skip_serializing_if = "Option::is_none")]
    pub multiplier: Option<f64>,
    #[serde(rename = "@offset", skip_serializing_if = "Option::is_none")]
    pub offset: Option<f64>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct SafetyController {
    #[serde(rename = "@soft_lower_limit", default)]
    pub soft_lower_limit: f64,
    #[serde(rename = "@soft_upper_limit", default)]
    pub soft_upper_limit: f64,
    #[serde(rename = "@k_position", default)]
    pub k_position: f64,
    #[serde(rename = "@k_velocity")]
    pub k_velocity: f64,
}

/// Urdf Joint element
/// See <http://wiki.ros.org/urdf/XML/joint> for more detail.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Joint {
    #[serde(rename = "@name")]
    pub name: String,
    #[serde(rename = "@type")]
    pub joint_type: JointType,
    #[serde(default)]
    pub origin: Pose,
    pub parent: LinkName,
    pub child: LinkName,
    #[serde(default)]
    pub axis: Axis,
    #[serde(default)]
    pub limit: JointLimit,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dynamics: Option<Dynamics>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mimic: Option<Mimic>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety_controller: Option<SafetyController>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Dynamics {
    #[serde(rename = "@damping", default)]
    pub damping: f64,
    #[serde(rename = "@friction", default)]
    pub friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, Deserialize, Serialize, Clone)]
#[serde(rename = "robot")]
pub struct Robot {
    #[serde(rename = "@name", default)]
    pub name: String,

    #[serde(rename = "link", default, skip_serializing_if = "Vec::is_empty")]
    pub links: Vec<Link>,

    #[serde(rename = "joint", default, skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<Joint>,

    #[serde(rename = "material", default, skip_serializing_if = "Vec::is_empty")]
    pub materials: Vec<Material>,
}
