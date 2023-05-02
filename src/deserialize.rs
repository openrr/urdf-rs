use yaserde::xml;
use yaserde::xml::attribute::OwnedAttribute;
use yaserde::xml::namespace::Namespace;
use yaserde::{YaDeserialize, YaSerialize};
use yaserde_derive::{YaDeserialize, YaSerialize};

use std::io::{Read, Write};

use std::collections::HashMap;
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

#[derive(Debug, Clone)]
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

// TODO(anyone) Derive YaSerialize and YaSerialize and remove custom impl once upstream bug is fixed
// https://github.com/media-io/yaserde/issues/129
impl YaSerialize for Geometry {
    fn serialize<W: Write>(
        &self,
        serializer: &mut yaserde::ser::Serializer<W>,
    ) -> Result<(), String> {
        serializer
            .write(xml::writer::XmlEvent::start_element("geometry"))
            .map_err(|e| e.to_string())?;
        match self {
            Geometry::Box { size } => {
                dbg!(&format!("{:.1} {:.1} {:.1}", size[0], size[1], size[2]));
                serializer
                    .write(
                        xml::writer::XmlEvent::start_element("box")
                            .attr("size", &format!("{} {} {}", size[0], size[1], size[2])),
                    )
                    .map_err(|e| e.to_string())?;
            }
            Geometry::Cylinder { radius, length } => {
                serializer
                    .write(
                        xml::writer::XmlEvent::start_element("cylinder")
                            .attr("radius", &radius.to_string())
                            .attr("length", &length.to_string()),
                    )
                    .map_err(|e| e.to_string())?;
            }
            Geometry::Capsule { radius, length } => {
                serializer
                    .write(
                        xml::writer::XmlEvent::start_element("capsule")
                            .attr("radius", &radius.to_string())
                            .attr("length", &length.to_string()),
                    )
                    .map_err(|e| e.to_string())?;
            }
            Geometry::Sphere { radius } => {
                serializer
                    .write(
                        xml::writer::XmlEvent::start_element("sphere")
                            .attr("radius", &radius.to_string()),
                    )
                    .map_err(|e| e.to_string())?;
            }
            Geometry::Mesh { filename, scale } => {
                let builder =
                    xml::writer::XmlEvent::start_element("mesh").attr("filename", filename);
                if let Some(scale) = scale {
                    serializer
                        .write(
                            builder
                                .attr("scale", &format!("{} {} {}", scale[0], scale[1], scale[2])),
                        )
                        .map_err(|e| e.to_string())?;
                } else {
                    serializer.write(builder).map_err(|e| e.to_string())?;
                }
            }
        }
        serializer
            .write(xml::writer::XmlEvent::end_element())
            .map_err(|e| e.to_string())?;
        serializer
            .write(xml::writer::XmlEvent::end_element())
            .map_err(|e| e.to_string())?;
        Ok(())
    }

    fn serialize_attributes(
        &self,
        attributes: Vec<OwnedAttribute>,
        namespace: Namespace,
    ) -> Result<(Vec<OwnedAttribute>, Namespace), String> {
        Ok((attributes, namespace))
    }
}

impl Vec3 {
    fn from_string(v: &String) -> Result<Self, String> {
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
        Ok(Vec3(res))
    }
}

impl YaDeserialize for Geometry {
    fn deserialize<R: Read>(
        deserializer: &mut yaserde::de::Deserializer<R>,
    ) -> Result<Self, String> {
        deserializer.next_event()?;
        if let Ok(xml::reader::XmlEvent::StartElement {
            name,
            attributes,
            namespace: _,
        }) = deserializer.peek()
        {
            let attributes: HashMap<_, _> = attributes
                .iter()
                .map(|attr| (attr.name.local_name.clone(), attr.value.clone()))
                .collect();
            if name.local_name == "box" {
                if let Some(v) = attributes.get("size") {
                    Ok(Self::Box {
                        size: Vec3::from_string(v)?,
                    })
                } else {
                    Err(format!(
                        "Failed parsing attributes for box {:?}",
                        attributes
                    ))
                }
            } else if name.local_name == "cylinder" {
                if let (Some(length), Some(radius)) = (
                    attributes.get("length").and_then(|a| a.parse::<f64>().ok()),
                    attributes.get("radius").and_then(|a| a.parse::<f64>().ok()),
                ) {
                    Ok(Self::Cylinder { length, radius })
                } else {
                    Err("Failed parsing radius and size for cylinder geometry".to_string())
                }
            } else if name.local_name == "capsule" {
                if let (Some(length), Some(radius)) = (
                    attributes.get("length").and_then(|a| a.parse::<f64>().ok()),
                    attributes.get("radius").and_then(|a| a.parse::<f64>().ok()),
                ) {
                    Ok(Self::Capsule { length, radius })
                } else {
                    Err("Failed parsing radius and size for capsule geometry".to_string())
                }
            } else if name.local_name == "sphere" {
                if let Some(radius) = attributes.get("radius").and_then(|a| a.parse::<f64>().ok()) {
                    Ok(Self::Sphere { radius })
                } else {
                    Err("Failed parsing radius and size for sphere geometry".to_string())
                }
            } else if name.local_name == "mesh" {
                if let Some(filename) = attributes
                    .get("filename")
                    .and_then(|a| a.parse::<String>().ok())
                {
                    let scale = attributes
                        .get("scale")
                        .and_then(|s| Vec3::from_string(s).ok());
                    Ok(Self::Mesh { filename, scale })
                } else {
                    Err("Error parsing filename for mesh".to_string())
                }
            } else {
                Err(format!("Invalid geometry name [{}] found", name.local_name))
            }
        } else {
            Err("Elements not found while parsing Geometry".to_string())
        }
    }
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Color {
    #[yaserde(attribute)]
    pub rgba: Vec4,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Texture {
    #[yaserde(attribute)]
    pub filename: String,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Material {
    #[yaserde(attribute)]
    pub name: String,
    pub color: Option<Color>,
    #[yaserde(attribute)]
    pub texture: Option<Texture>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Visual {
    #[yaserde(attribute)]
    pub name: Option<String>,
    pub origin: Pose,
    pub geometry: Geometry,
    pub material: Option<Material>,
}

#[derive(Debug, YaDeserialize, YaSerialize, Default, Clone)]
pub struct Collision {
    #[yaserde(attribute)]
    pub name: Option<String>,
    pub origin: Pose,
    pub geometry: Geometry,
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
pub struct Vec3(pub [f64; 3]);

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

impl YaSerialize for Vec3 {
    fn serialize<W: Write>(
        &self,
        serializer: &mut yaserde::ser::Serializer<W>,
    ) -> Result<(), String> {
        serializer
            .write(xml::writer::XmlEvent::Characters(&format!(
                "{} {} {}",
                self.0[0], self.0[1], self.0[2]
            )))
            .map_err(|e| e.to_string())
    }

    fn serialize_attributes(
        &self,
        attributes: Vec<OwnedAttribute>,
        namespace: Namespace,
    ) -> Result<(Vec<OwnedAttribute>, Namespace), String> {
        Ok((attributes, namespace))
    }
}

impl YaDeserialize for Vec3 {
    fn deserialize<R: Read>(
        deserializer: &mut yaserde::de::Deserializer<R>,
    ) -> Result<Self, String> {
        deserializer.next_event()?;
        if let Ok(xml::reader::XmlEvent::Characters(v)) = deserializer.peek() {
            Ok(Vec3::from_string(v)?)
        } else {
            Err("String of elements not found while parsing Vec3".to_string())
        }
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Vec4(pub [f64; 4]);

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

impl YaSerialize for Vec4 {
    fn serialize<W: Write>(
        &self,
        serializer: &mut yaserde::ser::Serializer<W>,
    ) -> Result<(), String> {
        serializer
            .write(xml::writer::XmlEvent::Characters(&format!(
                "{} {} {} {}",
                self.0[0], self.0[1], self.0[2], self.0[3]
            )))
            .map_err(|e| e.to_string())
    }

    fn serialize_attributes(
        &self,
        attributes: Vec<OwnedAttribute>,
        namespace: Namespace,
    ) -> Result<(Vec<OwnedAttribute>, Namespace), String> {
        Ok((attributes, namespace))
    }
}

impl YaDeserialize for Vec4 {
    fn deserialize<R: Read>(
        deserializer: &mut yaserde::de::Deserializer<R>,
    ) -> Result<Self, String> {
        deserializer.next_event()?;
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
            Ok(Vec4(res))
        } else {
            Err("String of elements not found while parsing Vec3".to_string())
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

#[derive(Debug, Default, YaDeserialize, YaSerialize, Clone, PartialEq, Eq)]
pub enum JointType {
    #[yaserde(rename = "revolute")]
    Revolute,
    #[yaserde(rename = "continuous")]
    Continuous,
    #[yaserde(rename = "prismatic")]
    Prismatic,
    #[default]
    #[yaserde(rename = "fixed")]
    Fixed,
    #[yaserde(rename = "floating")]
    Floating,
    #[yaserde(rename = "planar")]
    Planar,
    #[yaserde(rename = "spherical")]
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
    #[yaserde(attribute, rename = "type")]
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
