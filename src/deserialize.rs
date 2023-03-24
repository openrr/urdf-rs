use serde::{Deserialize, Serialize};
use serde::de::{Visitor};

use std::ops::{Deref, DerefMut};

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mass {
    pub value: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Inertial {
    #[serde(default)]
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum Geometry {
    Box {
        size: Vec3,
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
        #[serde(default)]
        scale: Option<Vec3>,
    },
}

impl Default for Geometry {
    fn default() -> Geometry {
        Geometry::Box {
            size: Vec3::default(),
        }
    }
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Color {
    pub rgba: Vec4,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Texture {
    pub filename: String,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Material {
    pub name: String,
    pub color: Option<Color>,
    pub texture: Option<Texture>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Visual {
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
    pub material: Option<Material>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Collision {
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
}

/// Urdf Link element
/// See <http://wiki.ros.org/urdf/XML/link> for more detail.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Link {
    pub name: String,
    #[serde(default)]
    pub inertial: Inertial,
    #[serde(default)]
    pub visual: Vec<Visual>,
    #[serde(default)]
    pub collision: Vec<Collision>,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Vec3([f64; 3]);

impl Deref for Vec3 {
    type Target = [f64; 3];

    fn deref(&self) -> &Self::Target {
        return &self.0;
    }
}

impl DerefMut for Vec3 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        return &mut self.0;
    }
}

impl Serialize for Vec3 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&format!("{} {} {}", self.0[0], self.0[1], self.0[2]))
    }
}

impl<'de> Deserialize<'de> for Vec3 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_str(Vec3Visitor)
    }
}

struct Vec3Visitor;
impl<'de> Visitor<'de> for Vec3Visitor {
    type Value = Vec3;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(
            "a string containing three floating point values separated by spaces",
        )
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let split_results: Vec<_> = v.split_whitespace().filter_map(|s| s.parse::<f64>().ok()).collect();
        if split_results.len() != 3 {
            return Err(E::custom(format!(
                "Wrong vector element count, expected 3 found {} for [{}]", split_results.len(), v)));
        }
        let mut res = [0.0f64; 3];
        res.copy_from_slice(&split_results);
        return Ok(Vec3(res));
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Vec4([f64; 4]);

impl Deref for Vec4 {
    type Target = [f64; 4];

    fn deref(&self) -> &Self::Target {
        return &self.0;
    }
}

impl DerefMut for Vec4 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        return &mut self.0;
    }
}

impl Serialize for Vec4 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&format!("{} {} {} {}", self.0[0], self.0[1], self.0[2], self.0[3]))
    }
}

impl<'de> Deserialize<'de> for Vec4 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_str(Vec4Visitor)
    }
}

struct Vec4Visitor;
impl<'de> Visitor<'de> for Vec4Visitor {
    type Value = Vec4;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(
            "a string containing four floating point values separated by spaces",
        )
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let split_results: Vec<_> = v.split_whitespace().filter_map(|s| s.parse::<f64>().ok()).collect();
        if split_results.len() != 4 {
            return Err(E::custom(format!(
                "Wrong vector element count, expected 4 found {} for [{}]", split_results.len(), v)));
        }
        let mut res = [0.0f64; 4];
        res.copy_from_slice(&split_results);
        return Ok(Vec4(res));
    }
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Axis {
    pub xyz: Vec3,
}

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub struct Pose {
    #[serde(default)]
    pub xyz: Vec3,
    #[serde(default)]
    pub rpy: Vec3,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct LinkName {
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
    #[serde(default)]
    pub lower: f64,
    #[serde(default)]
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
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
#[derive(Debug, Deserialize, Serialize, Clone)]
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

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Dynamics {
    #[serde(default)]
    pub damping: f64,
    #[serde(default)]
    pub friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, Deserialize, Serialize, Clone)]
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
