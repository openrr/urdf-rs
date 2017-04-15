#[macro_use]
extern crate serde_derive;
extern crate serde;
extern crate serde_xml_rs;

#[derive(Debug, Deserialize, Default)]
pub struct Mass {
    pub value: f64,
}

#[derive(Debug, Deserialize, Default)]
pub struct Inertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Debug, Deserialize, Default)]
pub struct Inertial {
    #[serde(default)]
    pub origin: Pose,
    pub mass: Mass,
    pub inertia: Inertia,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Geometry {
    Box {
        #[serde(with = "urdf_vec3")]
        size: [f64; 3]
        //size: String
    },
    Cylinder { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { filename: String, scale: f64 },
}

impl Default for Geometry {
    fn default() -> Geometry {
        Geometry::Box { size: [0.0f64, 0.0, 0.0] }
    }
}

#[derive(Debug, Deserialize, Default)]
pub struct Color {
    #[serde(with = "urdf_vec4")]
    #[serde(default="default_rgba")]
    pub rgba: [f64; 4],
}

#[derive(Debug, Deserialize)]
pub struct Texture {
    pub filename: String,
}

impl Default for Texture {
    fn default() -> Texture {
        Texture { filename: "".to_string() }
    }
}

#[derive(Debug, Deserialize, Default)]
pub struct Material {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub color: Color,
    #[serde(default)]
    pub texture: Texture,
}


#[derive(Debug, Deserialize, Default)]
pub struct Visual {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
    #[serde(default)]
    pub material: Material,
}


#[derive(Debug, Deserialize, Default)]
pub struct Collision {
    #[serde(default)]
    pub name: String,
    pub origin: Pose,
    pub geometry: Geometry,
}

/// Urdf Link element
/// See http://wiki.ros.org/urdf/XML/link for more detail.
#[derive(Debug, Deserialize)]
pub struct Link {
    pub name: String,
    #[serde(default)]
    pub inertial: Inertial,
    #[serde(default)]
    pub visual: Visual,
    #[serde(default)]
    pub collision: Collision,
}


#[derive(Deserialize, Debug)]
pub struct Vec3 {
    #[serde(with = "urdf_vec3")]
    pub data: [f64; 3]
}

mod urdf_vec3 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<D>(deserializer: D) -> Result<[f64; 3], D::Error>
    where D: Deserializer
{
        let s = String::deserialize(deserializer)?;
        let vec = s.split(" ").filter_map(|x| x.parse::<f64>().ok()).collect::<Vec<_>>();
        if vec.len() != 3 {
            return Err(serde::de::Error::custom(
                format!("failed to parse float array in {}", s)));
        }
        let mut arr = [0.0f64; 3];
        for i in 0..3 {
            arr[i] = vec[i];
        }
        Ok(arr)
    }
}

mod urdf_vec4 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<D>(deserializer: D) -> Result<[f64; 4], D::Error>
    where D: Deserializer
{
        let s = String::deserialize(deserializer)?;
        let vec = s.split(" ").filter_map(|x| x.parse::<f64>().ok()).collect::<Vec<_>>();
        if vec.len() != 4 {
            return Err(serde::de::Error::custom(
                format!("failed to parse float array in {}", s)));
        }
        let mut arr = [0.0f64; 4];
        for i in 0..4 {
            arr[i] = vec[i];
        }
        Ok(arr)
    }
}


#[derive(Debug, Deserialize)]
pub struct Axis {
    #[serde(with = "urdf_vec3")]
    pub xyz: [f64; 3]
}

impl Default for Axis {
    fn default() -> Axis {
        Axis { xyz: [1.0f64, 0.0, 0.0,] }
    }
}

#[derive(Debug, Deserialize)]
pub struct Pose {
    #[serde(with = "urdf_vec3")]
    #[serde(default="default_zero3")]
    pub xyz: [f64; 3],
    #[serde(with = "urdf_vec3")]
    #[serde(default="default_zero3")]
    pub rpy: [f64; 3],
}

fn default_zero3() -> [f64; 3] {
    [0.0f64, 0.0, 0.0]
}

fn default_rgba() -> [f64; 4] {
    [1.0f64, 1.0, 1.0, 1.0]
}

impl Default for Pose {
    fn default() -> Pose {
        Pose {
            xyz: default_zero3(),
            rpy: default_zero3(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct LinkName {
    pub link: String,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
}

#[derive(Debug, Deserialize)]
pub struct JointLimit {
    #[serde(default)]
    pub lower: f64,
    #[serde(default)]
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Default)]
pub struct Mimic {
    joint: String,
    multiplier: f64,
    offset: f64,
}

#[derive(Debug, Deserialize, Default)]
pub struct SafetyController {
    soft_lower_limit: f64,
    soft_upper_limit: f64,
    k_position: f64,
    k_velocity: f64,
}

/// Urdf Joint element
/// See http://wiki.ros.org/urdf/XML/joint for more detail.
#[derive(Debug, Deserialize)]
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
    pub limit: JointLimit,
    #[serde(default)]
    pub dynamics: Dynamics,
    #[serde(default)]
    pub mimic: Mimic,
    #[serde(default)]
    pub safety_controller: SafetyController,
}

#[derive(Debug, Deserialize, Default)]
pub struct Dynamics {
    damping: f64,
    friction: f64,
}

/// Top level struct to access urdf.
#[derive(Debug, Deserialize)]
pub struct Robot {
    pub name: String,

    #[serde(rename = "link", default)]
    pub links: Vec<Link>,

    #[serde(rename = "joint", default)]
    pub joints: Vec<Joint>,
}

use std::fs::File;
use std::io::prelude::*;
use std::path::Path;
use std::error::Error;
use std::fmt;

#[derive(Debug)]
pub enum UrdfError {
    File(std::io::Error),
    Xml(serde_xml_rs::Error),
    Parse(String),
}

impl fmt::Display for UrdfError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            UrdfError::File(ref err) => err.fmt(f),
            UrdfError::Xml(ref err) => err.fmt(f),
            UrdfError::Parse(ref msg) => write!(f, "parse error {}", msg),
        }
    }
}

impl Error for UrdfError {
    fn description(&self) -> &str {
        match *self {
            UrdfError::File(ref err) => err.description(),
            UrdfError::Xml(ref err) => err.description(),
            UrdfError::Parse(_) => "parse error",
        }
    }
}

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

/// Read urdf file and create Robot instance
///
/// # Examples
///
/// ```
/// extern crate urdf_rs;
/// let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
/// let links = urdf_robo.links;
/// ```
pub fn read_file<P: AsRef<Path>>(path: P) -> Result<Robot, UrdfError> {
    let mut file = File::open(path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    read_from_string(&contents)
}


/// Read from string instead of file.
///
///
/// # Examples
///
/// ```
/// let s = r##"
///     <robot name="robo">
///         <link name="shoulder1">
///             <inertial>
///                 <origin xyz="0 0 0.5" rpy="0 0 0"/>
///                 <mass value="1"/>
///                 <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
///             </inertial>
///             <visual>
///                 <origin xyz="0.1 0.2 0.3" rpy="-0.1 -0.2  -0.3" />
///                 <geometry>
///                     <box size="1.0 2.0 3.0" />
///                 </geometry>
///                 <material name="Cyan">
///                     <color rgba="0 1.0 1.0 1.0"/>
///                 </material>
///             </visual>
///             <collision>
///                 <origin xyz="0 0 0" rpy="0 0 0"/>
///                 <geometry>
///                     <cylinder radius="1" length="0.5"/>
///                 </geometry>
///             </collision>
///         </link>
///         <link name="elbow1" />
///         <link name="wrist1" />
///         <joint name="shoulder_pitch" type="revolute">
///             <origin xyz="0.0 0.0 0.1" />
///             <parent link="shoulder1" />
///             <child link="elbow1" />
///             <axis xyz="0 1 -1" />
///             <limit lower="-1" upper="1.0" effort="0" velocity="1.0"/>
///         </joint>
///         <joint name="shoulder_pitch" type="revolute">
///             <origin xyz="0.0 0.0 0.0" />
///             <parent link="elbow1" />
///             <child link="wrist1" />
///             <axis xyz="0 1 0" />
///             <limit lower="-2" upper="1.0" effort="0" velocity="1.0"/>
///         </joint>
///     </robot>
///    "##;
/// let robo = urdf_rs::read_from_string(s).unwrap();
/// ```

pub fn read_from_string(string: &str) -> Result<Robot, UrdfError> {
    serde_xml_rs::deserialize(string.as_bytes())
        .map_err(From::from)
}

#[test]
fn it_works() {
    let s = r##"
        <robot name="robo">
            <link name="shoulder1">
                <inertial>
                    <origin xyz="0 0 0.5" rpy="0 0 0"/>
                    <mass value="1"/>
                    <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
                </inertial>
                <visual>
                    <origin xyz="0.1 0.2 0.3" rpy="-0.1 -0.2  -0.3" />
                    <geometry>
                        <box size="1.0 2.0 3.0" />
                    </geometry>
                    <material name="Cyan">
                        <color rgba="0 1.0 1.0 1.0"/>
                    </material>
                </visual>
                <collision>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <cylinder radius="1" length="0.5"/>
                    </geometry>
                </collision>
            </link>
            <link name="elbow1" />
            <link name="wrist1" />
            <joint name="shoulder_pitch" type="revolute">
                <origin xyz="0.0 0.0 0.1" />
                <parent link="shoulder1" />
                <child link="elbow1" />
                <axis xyz="0 1 -1" />
                <limit lower="-1" upper="1.0" effort="0" velocity="1.0"/>
            </joint>
            <joint name="shoulder_pitch" type="revolute">
                <origin xyz="0.0 0.0 0.0" />
                <parent link="elbow1" />
                <child link="wrist1" />
                <axis xyz="0 1 0" />
                <limit lower="-2" upper="1.0" effort="0" velocity="1.0"/>
            </joint>
        </robot>
    "##;
    let robo = read_from_string(s).unwrap();

    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 3);
    assert_eq!(robo.joints.len(), 2);
    let xyz = robo.links[0].visual.origin.xyz;
    assert_eq!(xyz[0], 0.1);
    assert_eq!(xyz[1], 0.2);
    assert_eq!(xyz[2], 0.3);
    let rpy = robo.links[0].visual.origin.rpy;
    assert_eq!(rpy[0], -0.1);
    assert_eq!(rpy[1], -0.2);
    assert_eq!(rpy[2], -0.3);

    match robo.links[0].visual.geometry {
        Geometry::Box{size} => {
            assert_eq!(size[0], 1.0f64);
            assert_eq!(size[1], 2.0f64);
            assert_eq!(size[2], 3.0f64);
        },
        _ => { panic!("geometry error") },
    }

    assert_eq!(robo.joints[0].name, "shoulder_pitch");
    let xyz = robo.joints[0].axis.xyz;
    assert_eq!(xyz[0], 0.0f64);
    assert_eq!(xyz[1], 1.0f64);
    assert_eq!(xyz[2], -1.0f64);
    let xyz = robo.joints[0].axis.xyz;
    //"0 1 -1"
    assert_eq!(xyz[0], 0.0);
    assert_eq!(xyz[1], 1.0);
    assert_eq!(xyz[2], -1.0);

}
