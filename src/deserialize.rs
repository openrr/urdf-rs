use yaserde::{YaSerialize, YaDeserialize, Visitor};
use yaserde::xml::namespace::Namespace;
use yaserde::ser::Serializer;
use yaserde::xml::attribute::OwnedAttribute;
use yaserde::xml;
use yaserde_derive::{YaSerialize, YaDeserialize};

use std::io::{Read, Write};

use std::ops::{Deref, DerefMut};

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Mass {
    #[yaserde(attribute)]
    pub value: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Inertia {
    #[yaserde(attribute)]
    pub ixx: f64,
    #[yaserde(attribute)]
    pub ixy: f64,
    #[yaserde(attribute)]
    pub ixz: f64,
    #[yaserde(attribute)]
    pub iyy: f64,
    #[yaserde(attribute)]
    pub iyz: f64,
    #[yaserde(attribute)]
    pub izz: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Inertial {
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct BoxGeometry {
    #[yaserde(attribute)]
    pub size: Vec3,
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct CylinderGeometry {
    #[yaserde(attribute)]
    pub radius: f64,
    #[yaserde(attribute)]
    pub length: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct CapsuleGeometry {
    #[yaserde(attribute)]
    pub radius: f64,
    #[yaserde(attribute)]
    pub length: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct SphereGeometry {
    #[yaserde(attribute)]
    pub radius: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct MeshGeometry {
    #[yaserde(attribute)]
    pub filename: String,
    #[yaserde(attribute)]
    pub scale: Option<Vec3>,
}

pub enum Geometry {
    Box(BoxGeometry),
    Cylinder(CylinderGeometry),
    Capsule(CapsuleGeometry),
    Sphere(SphereGeometry),
    Mesh(MeshGeometry),
}

#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct GeometrySerde {
    #[yaserde(rename = "box")]
    pub box_geometry: Option<BoxGeometry>,
    #[yaserde(rename = "cylinder")]
    pub cylinder: Option<CylinderGeometry>,
    #[yaserde(rename = "capsule")]
    pub capsule: Option<CapsuleGeometry>,
    #[yaserde(rename = "sphere")]
    pub sphere: Option<SphereGeometry>,
    #[yaserde(rename = "mesh")]
    pub mesh: Option<MeshGeometry>,
}

impl From<&GeometrySerde> for Geometry {
    fn from(geom: &GeometrySerde) -> Geometry {
        if let Some(b) = &geom.box_geometry {
            Geometry::Box(b.clone())
        } else if let Some(c) = &geom.cylinder {
            Geometry::Cylinder(c.clone())
        } else if let Some(c) = &geom.capsule {
            Geometry::Capsule(c.clone())
        } else if let Some(s) = &geom.sphere {
            Geometry::Sphere(s.clone())
        } else if let Some(m) = &geom.mesh {
            Geometry::Mesh(m.clone())
        } else {
            panic!("Invalid geometry serde structure");
        }
    }
}

impl Default for GeometrySerde {
    fn default() -> Self {
        Self {
            box_geometry: Some(BoxGeometry{size: Vec3::default()}),
            cylinder: None,
            capsule: None,
            sphere: None,
            mesh: None,
        }
    }
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Color {
    pub rgba: Vec4,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Texture {
    pub filename: String,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Material {
    #[yaserde(attribute)]
    pub name: String,
    pub color: Option<Color>,
    pub texture: Option<Texture>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Visual {
    pub name: Option<String>,
    pub origin: Pose,
    pub geometry: GeometrySerde,
    pub material: Option<Material>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Collision {
    pub name: Option<String>,
    pub origin: Pose,
    pub geometry: GeometrySerde,
}

/// Urdf Link element
/// See <http://wiki.ros.org/urdf/XML/link> for more detail.
#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct Link {
    #[yaserde(attribute)]
    pub name: String,
    pub inertial: Inertial,
    pub visual: Vec<Visual>,
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

impl YaSerialize for Vec3 {
    fn serialize<W: Write>(&self, serializer: &mut yaserde::ser::Serializer<W>) -> Result<(), String>
    {
        // TODO(luca) cleanup this
        println!("Serializing {:?}", self);
        match serializer.write(xml::writer::XmlEvent::Characters(&format!("{} {} {}", self.0[0], self.0[1], self.0[2]))) {
            Ok(()) => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }

    // TODO(luca) check this implementation
    fn serialize_attributes(
        &self, 
        attributes: Vec<OwnedAttribute>, 
        namespace: Namespace
    ) -> Result<(Vec<OwnedAttribute>, Namespace), String> {
        Ok((attributes, namespace))
    }
}

impl YaDeserialize for Vec3 {
    fn deserialize<R: Read>(deserializer: &mut yaserde::de::Deserializer<R>) -> Result<Self, String>
    {
        deserializer.next_event();
        if let Ok(xml::reader::XmlEvent::Characters(v)) = deserializer.peek() {
            let split_results: Vec<_> = v
                .split_whitespace()
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
            if split_results.len() != 3 {
                return Err(format!(
                    "Wrong vector element count, expected 3 found {} for [{}]",
                    split_results.len(),
                    v
                ));
            }
            let mut res = [0.0f64; 3];
            res.copy_from_slice(&split_results);
            return Ok(Vec3(res));
        } else {
            return Err("String of elements not found while parsing Vec3".to_string());
        }
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

impl YaSerialize for Vec4 {
    fn serialize<W: Write>(&self, serializer: &mut yaserde::ser::Serializer<W>) -> Result<(), String>
    {
        // TODO(luca) cleanup this
        match serializer.write(xml::writer::XmlEvent::Characters(&format!("{} {} {} {}", self.0[0], self.0[1], self.0[2], self.0[3]))) {
            Ok(()) => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }

    // TODO(luca) check this implementation
    fn serialize_attributes(
        &self, 
        attributes: Vec<OwnedAttribute>, 
        namespace: Namespace
    ) -> Result<(Vec<OwnedAttribute>, Namespace), String> {
        Ok((attributes, namespace))
    }
}

impl YaDeserialize for Vec4 {
    fn deserialize<R: Read>(deserializer: &mut yaserde::de::Deserializer<R>) -> Result<Self, String>
    {
        deserializer.next_event();
        if let xml::reader::XmlEvent::Characters(v) = deserializer.peek()? {
            let split_results: Vec<_> = v
                .split_whitespace()
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
            if split_results.len() != 4 {
                return Err(format!(
                    "Wrong vector element count, expected 4 found {} for [{}]",
                    split_results.len(),
                    v
                ));
            }
            let mut res = [0.0f64; 4];
            res.copy_from_slice(&split_results);
            return Ok(Vec4(res));
        } else {
            return Err("String of elements not found while parsing Vec3".to_string());
        }
    }
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Axis {
    #[yaserde(attribute)]
    pub xyz: Vec3,
}

#[derive(Debug, Default, YaDeserialize, YaSerialize, Clone)]
pub struct Pose {
    #[yaserde(attribute)]
    pub xyz: Vec3,
    #[yaserde(attribute)]
    pub rpy: Vec3,
}

#[derive(Debug, Default, YaDeserialize, YaSerialize, Clone)]
pub struct LinkName {
    #[yaserde(attribute)]
    pub link: String,
}

// TODO(luca) see if we can avoid deriving default
#[derive(Debug, Default, YaDeserialize, YaSerialize, Clone, PartialEq, Eq)]
#[yaserde(rename_all = "snake_case")]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    #[default]
    Fixed,
    Floating,
    Planar,
    Spherical,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct JointLimit {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct SafetyController {
    pub soft_lower_limit: f64,
    pub soft_upper_limit: f64,
    pub k_position: f64,
    pub k_velocity: f64,
}

/// Urdf Joint element
/// See <http://wiki.ros.org/urdf/XML/joint> for more detail.
#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct Joint {
    #[yaserde(attribute)]
    pub name: String,
    #[yaserde(rename = "type")]
    pub joint_type: JointType,
    pub origin: Pose,
    pub parent: LinkName,
    pub child: LinkName,
    pub axis: Axis,
    pub limit: JointLimit,
    pub dynamics: Option<Dynamics>,
    pub mimic: Option<Mimic>,
    pub safety_controller: Option<SafetyController>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Dynamics {
    pub damping: f64,
    pub friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, YaDeserialize, YaSerialize, Clone)]
pub struct Robot {
    #[yaserde(attribute)]
    pub name: String,

    #[yaserde(rename = "link")]
    pub links: Vec<Link>,

    #[yaserde(rename = "joint")]
    pub joints: Vec<Joint>,

    #[yaserde(rename = "material")]
    pub materials: Vec<Material>,
}
