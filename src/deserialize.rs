use serde::de::Visitor;
use serde::{Deserialize, Serialize};

use std::ops::{Deref, DerefMut};

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mass {
    #[serde(rename(serialize = "@value"))]
    pub value: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Inertia {
    #[serde(rename(serialize = "@ixx"))]
    pub ixx: f64,
    #[serde(rename(serialize = "@ixy"))]
    pub ixy: f64,
    #[serde(rename(serialize = "@ixz"))]
    pub ixz: f64,
    #[serde(rename(serialize = "@iyy"))]
    pub iyy: f64,
    #[serde(rename(serialize = "@iyz"))]
    pub iyz: f64,
    #[serde(rename(serialize = "@izz"))]
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
        #[serde(rename(serialize = "@size"))]
        size: Vec3,
    },
    Cylinder {
        #[serde(rename(serialize = "@radius"))]
        radius: f64,
        #[serde(rename(serialize = "@length"))]
        length: f64,
    },
    Capsule {
        #[serde(rename(serialize = "@radius"))]
        radius: f64,
        #[serde(rename(serialize = "@length"))]
        length: f64,
    },
    Sphere {
        #[serde(rename(serialize = "@radius"))]
        radius: f64,
    },
    Mesh {
        #[serde(rename(serialize = "@filename"))]
        filename: String,
        #[serde(rename(serialize = "@scale"), skip_serializing_if = "Option::is_none")]
        scale: Option<Vec3>,
    },
}

impl Default for Geometry {
    fn default() -> Self {
        Self::Box {
            size: Vec3::default(),
        }
    }
}

#[derive(Debug, Default, Serialize, Clone)]
pub struct GeometryTag {
    #[serde(rename(serialize = "$value"))]
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

impl<'de> Deserialize<'de> for GeometryTag {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        Ok(Self {
            value: Geometry::deserialize(deserializer)?,
        })
    }
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Color {
    #[serde(rename(serialize = "@rgba"))]
    pub rgba: Vec4,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Texture {
    #[serde(rename(serialize = "@filename"))]
    pub filename: String,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Material {
    #[serde(rename(serialize = "@name"))]
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<Color>,
    #[serde(
        rename(serialize = "@texture"),
        skip_serializing_if = "Option::is_none"
    )]
    pub texture: Option<Texture>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Visual {
    #[serde(rename(serialize = "@name"), skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: GeometryTag,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material: Option<Material>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Collision {
    #[serde(rename(serialize = "@name"), skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: GeometryTag,
}

/// Urdf Link element
/// See <http://wiki.ros.org/urdf/XML/link> for more detail.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Link {
    #[serde(rename(serialize = "@name"))]
    pub name: String,
    #[serde(default)]
    pub inertial: Inertial,
    #[serde(default)]
    pub visual: Vec<Visual>,
    #[serde(default)]
    pub collision: Vec<Collision>,
}

#[derive(Debug, Serialize, Default, Clone, Copy, PartialEq)]
pub struct Vec3(#[serde(rename(serialize = "$text"))] pub [f64; 3]);

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

impl<'de> Deserialize<'de> for Vec3 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_str(Vec3Visitor)
    }
}

struct Vec3Visitor;
impl Visitor<'_> for Vec3Visitor {
    type Value = Vec3;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("a string containing three floating point values separated by spaces")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let split_results: Vec<_> = v
            .split_whitespace()
            .filter_map(|s| s.parse::<f64>().ok())
            .collect();
        if split_results.len() != 3 {
            return Err(E::custom(format!(
                "Wrong vector element count, expected 3 found {} for [{}]",
                split_results.len(),
                v
            )));
        }
        let mut res = [0.0f64; 3];
        res.copy_from_slice(&split_results);
        Ok(Vec3(res))
    }
}

#[derive(Debug, Serialize, Default, Clone, Copy, PartialEq)]
pub struct Vec4(#[serde(rename(serialize = "$text"))] pub [f64; 4]);

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

impl<'de> Deserialize<'de> for Vec4 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_str(Vec4Visitor)
    }
}

struct Vec4Visitor;
impl Visitor<'_> for Vec4Visitor {
    type Value = Vec4;

    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("a string containing four floating point values separated by spaces")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let split_results: Vec<_> = v
            .split_whitespace()
            .filter_map(|s| s.parse::<f64>().ok())
            .collect();
        if split_results.len() != 4 {
            return Err(E::custom(format!(
                "Wrong vector element count, expected 4 found {} for [{}]",
                split_results.len(),
                v
            )));
        }
        let mut res = [0.0f64; 4];
        res.copy_from_slice(&split_results);
        Ok(Vec4(res))
    }
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Axis {
    #[serde(rename(serialize = "@xyz"))]
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
    #[serde(rename(serialize = "@xyz"), default)]
    pub xyz: Vec3,
    #[serde(rename(serialize = "@rpy"), default)]
    pub rpy: Vec3,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct LinkName {
    #[serde(rename(serialize = "@link"))]
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
    #[serde(rename(serialize = "@lower"), default)]
    pub lower: f64,
    #[serde(rename(serialize = "@upper"), default)]
    pub upper: f64,
    #[serde(rename(serialize = "@effort"), default)]
    pub effort: f64,
    #[serde(rename(serialize = "@velocity"))]
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct Mimic {
    #[serde(rename(serialize = "@joint"))]
    pub joint: String,
    #[serde(
        rename(serialize = "@multiplier"),
        skip_serializing_if = "Option::is_none"
    )]
    pub multiplier: Option<f64>,
    #[serde(rename(serialize = "@offset"), skip_serializing_if = "Option::is_none")]
    pub offset: Option<f64>,
}

#[derive(Debug, Deserialize, Serialize, Default, Clone)]
pub struct SafetyController {
    #[serde(rename(serialize = "@soft_lower_limit"), default)]
    pub soft_lower_limit: f64,
    #[serde(rename(serialize = "@soft_upper_limit"), default)]
    pub soft_upper_limit: f64,
    #[serde(rename(serialize = "@k_position"), default)]
    pub k_position: f64,
    #[serde(rename(serialize = "@k_velocity"))]
    pub k_velocity: f64,
}

/// Urdf Joint element
/// See <http://wiki.ros.org/urdf/XML/joint> for more detail.
#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Joint {
    #[serde(rename(serialize = "@name"))]
    pub name: String,
    #[serde(rename(deserialize = "type", serialize = "@type"))]
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
    #[serde(rename(serialize = "@damping"), default)]
    pub damping: f64,
    #[serde(rename(serialize = "@friction"), default)]
    pub friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, Deserialize, Serialize, Clone)]
#[serde(rename = "robot")]
pub struct Robot {
    #[serde(rename(serialize = "@name"), default)]
    pub name: String,

    #[serde(rename = "link", default, skip_serializing_if = "Vec::is_empty")]
    pub links: Vec<Link>,

    #[serde(rename = "joint", default, skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<Joint>,

    #[serde(rename = "material", default, skip_serializing_if = "Vec::is_empty")]
    pub materials: Vec<Material>,
}
