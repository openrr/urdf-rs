#[derive(Debug, Deserialize, Default, Clone)]
pub struct Mass {
    pub value: f64,
}

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
        #[serde(with = "urdf_vec3")]
        size: [f64; 3],
    },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh {
        filename: String,
        #[serde(default = "default_scale")]
        #[serde(with = "urdf_vec3")]
        scale: [f64; 3],
    },
}

fn default_scale() -> [f64; 3] {
    [1.0f64; 3]
}

fn default_one() -> f64 {
    1.0f64
}

impl Default for Geometry {
    fn default() -> Geometry {
        Geometry::Box { size: [0.0f64, 0.0, 0.0] }
    }
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Color {
    #[serde(with = "urdf_vec4")]
    #[serde(default = "default_rgba")]
    pub rgba: [f64; 4],
}

#[derive(Debug, Deserialize, Clone)]
pub struct Texture {
    pub filename: String,
}

impl Default for Texture {
    fn default() -> Texture {
        Texture { filename: "".to_string() }
    }
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Material {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub color: Color,
    #[serde(default)]
    pub texture: Texture,
}


#[derive(Debug, Deserialize, Default, Clone)]
pub struct Visual {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
    #[serde(default)]
    pub material: Material,
}


#[derive(Debug, Deserialize, Default, Clone)]
pub struct Collision {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub origin: Pose,
    pub geometry: Geometry,
}

/// Urdf Link element
/// See http://wiki.ros.org/urdf/XML/link for more detail.
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


#[derive(Deserialize, Debug, Clone)]
pub struct Vec3 {
    #[serde(with = "urdf_vec3")]
    pub data: [f64; 3],
}

mod urdf_vec3 {
    use serde::{self, Deserialize, Deserializer};
    pub fn deserialize<D>(deserializer: D) -> Result<[f64; 3], D::Error>
    where
        D: Deserializer,
    {
        let s = String::deserialize(deserializer)?;
        let vec = s.split(' ')
            .filter_map(|x| x.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if vec.len() != 3 {
            return Err(serde::de::Error::custom(
                format!("failed to parse float array in {}", s),
            ));
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
    where
        D: Deserializer,
    {
        let s = String::deserialize(deserializer)?;
        let vec = s.split(' ')
            .filter_map(|x| x.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if vec.len() != 4 {
            return Err(serde::de::Error::custom(
                format!("failed to parse float array in {}", s),
            ));
        }
        let mut arr = [0.0f64; 4];
        for i in 0..4 {
            arr[i] = vec[i];
        }
        Ok(arr)
    }
}


#[derive(Debug, Deserialize, Clone)]
pub struct Axis {
    #[serde(with = "urdf_vec3")]
    pub xyz: [f64; 3],
}

impl Default for Axis {
    fn default() -> Axis {
        Axis { xyz: [1.0f64, 0.0, 0.0] }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct Pose {
    #[serde(with = "urdf_vec3")]
    #[serde(default = "default_zero3")]
    pub xyz: [f64; 3],
    #[serde(with = "urdf_vec3")]
    #[serde(default = "default_zero3")]
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

#[derive(Debug, Deserialize, Clone)]
pub struct LinkName {
    pub link: String,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Free,
    Spherical,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct JointLimit {
    #[serde(default)]
    pub lower: f64,
    #[serde(default)]
    pub upper: f64,
    #[serde(default)]
    pub effort: f64,
    #[serde(default)]
    pub velocity: f64,
}

#[derive(Debug, Deserialize, Default, Clone)]
pub struct Mimic {
    pub joint: String,
    #[serde(default = "default_one")]
    pub multiplier: f64,
    #[serde(default)]
    pub offset: f64,
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
/// See http://wiki.ros.org/urdf/XML/joint for more detail.
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
    #[serde(default)]
    pub dynamics: Dynamics,
    #[serde(default)]
    pub mimic: Mimic,
    #[serde(default)]
    pub safety_controller: SafetyController,
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
