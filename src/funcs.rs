use crate::deserialize::*;
use crate::errors::*;
use serde::Serialize;

use std::mem;
use std::path::Path;

/// URDF version specified by the `version` attribute of `<robot>`.
///
/// The derived ordering compares `major` first, then `minor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct UrdfVersion {
    major: u32,
    minor: u32,
}

impl UrdfVersion {
    const V1_0: Self = Self::new(1, 0);
    /// Adds `quat_xyzw` of `<origin>`.
    const V1_1: Self = Self::new(1, 1);
    /// Adds `acceleration`, `deceleration` and `jerk` of `<limit>`, and
    /// changes the defaults and validation of limits and geometries.
    const V1_2: Self = Self::new(1, 2);

    const fn new(major: u32, minor: u32) -> Self {
        Self { major, minor }
    }

    /// Parses the `version` attribute of `<robot>`.
    ///
    /// Like urdfdom, a missing attribute means version 1.0. A malformed value
    /// is also treated as version 1.0.
    fn from_robot(robot: &xml::Element) -> Self {
        robot
            .get_attribute("version", None)
            .and_then(|v| {
                let (major, minor) = v.trim().split_once('.')?;
                Some(Self::new(major.parse().ok()?, minor.parse().ok()?))
            })
            .unwrap_or(Self::V1_0)
    }
}

/// Converts a quaternion (x, y, z, w) to roll, pitch, yaw.
///
/// This follows `urdf::Rotation::getRPY` of urdfdom_headers.
fn quaternion_to_rpy([x, y, z, w]: [f64; 4]) -> [f64; 3] {
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    let [x, y, z, w] = if norm > 0.0 {
        [x / norm, y / norm, z / norm, w / norm]
    } else {
        [0.0, 0.0, 0.0, 1.0]
    };
    let sin_pitch = -2.0 * (x * z - w * y);
    if sin_pitch <= -0.99999 {
        [0.0, -std::f64::consts::FRAC_PI_2, 2.0 * x.atan2(-y)]
    } else if sin_pitch >= 0.99999 {
        [0.0, std::f64::consts::FRAC_PI_2, 2.0 * (-x).atan2(y)]
    } else {
        let (sqx, sqy, sqz, sqw) = (x * x, y * y, z * z, w * w);
        [
            (2.0 * (y * z + w * x)).atan2(sqw - sqx - sqy + sqz),
            sin_pitch.asin(),
            (2.0 * (x * y + w * z)).atan2(sqw + sqx - sqy - sqz),
        ]
    }
}

/// Handles the `quat_xyzw` attribute of `<origin>` introduced in URDF 1.1.
///
/// For URDF 1.1 or later, `quat_xyzw` is converted to `rpy` (specifying both
/// is an error). For older versions, `quat_xyzw` is ignored, like urdfdom.
fn convert_quat_xyzw(elm: &mut xml::Element, version: UrdfVersion) -> Result<()> {
    if let Some(quat) = elm.remove_attribute("quat_xyzw", None) {
        if version >= UrdfVersion::V1_1 {
            if elm.get_attribute("rpy", None).is_some() {
                return Err(
                    "Both rpy and quat_xyzw orientations are defined. Use either one or the other."
                        .into(),
                );
            }
            let values = quat
                .split_whitespace()
                .map(str::parse::<f64>)
                .collect::<std::result::Result<Vec<_>, _>>()
                .ok()
                .and_then(|v| <[f64; 4]>::try_from(v).ok())
                .ok_or_else(|| format!("quat_xyzw must be four floating point values: [{quat}]"))?;
            let [r, p, y] = quaternion_to_rpy(values);
            elm.set_attribute("rpy".to_owned(), None, format!("{r} {p} {y}"));
        }
    }
    Ok(())
}

/// Parses an optional attribute as `f64`. NaN is an error.
fn get_f64_attribute(elm: &xml::Element, name: &str, context: &str) -> Result<Option<f64>> {
    elm.get_attribute(name, None)
        .map(|v| match v.trim().parse::<f64>() {
            Ok(value) if !value.is_nan() => Ok(value),
            _ => Err(format!("{context}: {name} value ({v}) is not a valid float").into()),
        })
        .transpose()
}

/// Handles `<limit>` of `<joint>` according to the URDF version.
///
/// For URDF 1.2 or later, omitted limits mean no limit (infinity), and
/// negative limits and `upper < lower` are errors. `deceleration` defaults to
/// `acceleration`. For older versions, `acceleration`, `deceleration` and
/// `jerk` introduced in URDF 1.2 are ignored, like urdfdom.
fn convert_joint_limit(elm: &mut xml::Element, version: UrdfVersion, joint: &str) -> Result<()> {
    const ATTRS_SINCE_1_2: [&str; 3] = ["acceleration", "deceleration", "jerk"];
    if version < UrdfVersion::V1_2 {
        for name in ATTRS_SINCE_1_2 {
            elm.remove_attribute(name, None);
        }
        return Ok(());
    }

    let context = format!("joint [{joint}]");
    let lower = get_f64_attribute(elm, "lower", &context)?;
    let upper = get_f64_attribute(elm, "upper", &context)?;
    if let (Some(lower), Some(upper)) = (lower, upper) {
        if upper < lower {
            return Err(format!(
                "{context}: upper position limit ({upper}) cannot be smaller than lower position limit ({lower})"
            )
            .into());
        }
    }
    for name in ["effort", "velocity"].into_iter().chain(ATTRS_SINCE_1_2) {
        if let Some(value) = get_f64_attribute(elm, name, &context)? {
            if value < 0.0 {
                return Err(format!("{context}: {name} value ({value}) is negative").into());
            }
        }
    }

    for (name, default) in [
        ("lower", "-inf"),
        ("upper", "inf"),
        ("effort", "inf"),
        ("velocity", "inf"),
    ] {
        if elm.get_attribute(name, None).is_none() {
            elm.set_attribute(name.to_owned(), None, default.to_owned());
        }
    }
    if elm.get_attribute("deceleration", None).is_none() {
        if let Some(acceleration) = elm.get_attribute("acceleration", None) {
            let acceleration = acceleration.to_owned();
            elm.set_attribute("deceleration".to_owned(), None, acceleration);
        }
    }
    Ok(())
}

/// Checks that the dimensions of `<sphere>`, `<box>`, `<cylinder>` and
/// `<capsule>` are positive finite values, as required by URDF 1.2 or later.
fn check_geometry(elm: &xml::Element) -> Result<()> {
    let (shape, attrs): (_, &[_]) = match &*elm.name {
        "sphere" => ("Sphere", &["radius"]),
        "box" => ("Box", &["size"]),
        "cylinder" => ("Cylinder", &["radius", "length"]),
        "capsule" => ("Capsule", &["radius", "length"]),
        _ => return Ok(()),
    };
    for attr in attrs {
        // Missing or malformed values are reported by the deserializer.
        let Some(value) = elm.get_attribute(attr, None) else {
            continue;
        };
        let valid = value
            .split_whitespace()
            .filter_map(|v| v.parse::<f64>().ok())
            .all(|v| v.is_finite() && v > 0.0);
        if !valid {
            return Err(format!("{shape} {attr} must be positive finite values: [{value}]").into());
        }
    }
    Ok(())
}

/// Handles the elements and attributes whose behavior depends on the URDF
/// version.
fn convert_versioned(elm: &mut xml::Element, version: UrdfVersion) -> Result<()> {
    let joint_name = (elm.name == "joint").then(|| {
        elm.get_attribute("name", None)
            .unwrap_or_default()
            .to_owned()
    });
    let is_geometry = elm.name == "geometry";
    for c in &mut elm.children {
        let xml::Xml::ElementNode(child) = c else {
            continue;
        };
        match (&joint_name, &*child.name) {
            (_, "origin") => convert_quat_xyzw(child, version)?,
            (Some(joint), "limit") => convert_joint_limit(child, version, joint)?,
            _ if is_geometry && version >= UrdfVersion::V1_2 => check_geometry(child)?,
            _ => {}
        }
        convert_versioned(child, version)?;
    }
    Ok(())
}

/// sort <link> and <joint> to avoid the [issue](https://github.com/RReverser/serde-xml-rs/issues/5)
fn sort_link_joint(string: &str) -> Result<String> {
    let mut e: xml::Element = string.parse().map_err(UrdfError::new)?;
    let version = UrdfVersion::from_robot(&e);
    convert_versioned(&mut e, version)?;
    let mut links = Vec::new();
    let mut joints = Vec::new();
    let mut materials = Vec::new();
    for c in mem::take(&mut e.children) {
        if let xml::Xml::ElementNode(xml_elm) = c {
            if xml_elm.name == "link" {
                links.push(sort_visual_collision(xml_elm));
            } else if xml_elm.name == "joint" {
                joints.push(xml::Xml::ElementNode(xml_elm));
            } else if xml_elm.name == "material" {
                materials.push(xml::Xml::ElementNode(xml_elm));
            }
        }
    }
    let mut new_elm = e;
    links.extend(joints);
    links.extend(materials);
    new_elm.children = links;
    Ok(format!("{new_elm}"))
}

fn sort_visual_collision(mut elm: xml::Element) -> xml::Xml {
    let mut visuals = Vec::new();
    let mut collisions = Vec::new();
    for c in mem::take(&mut elm.children) {
        if let xml::Xml::ElementNode(xml_elm) = c {
            if xml_elm.name == "visual" || xml_elm.name == "inertial" {
                visuals.push(xml::Xml::ElementNode(xml_elm));
            } else if xml_elm.name == "collision" {
                collisions.push(xml::Xml::ElementNode(xml_elm));
            }
        }
    }
    let mut new_elm = elm;
    visuals.extend(collisions);
    new_elm.children = visuals;
    xml::Xml::ElementNode(new_elm)
}

/// Read urdf file and create Robot instance
///
/// # Examples
///
/// ```
/// let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
/// let links = urdf_robot.links;
/// println!("{:?}", links[0].visual[0].origin.xyz);
/// ```
pub fn read_file<P: AsRef<Path>>(path: P) -> Result<Robot> {
    read_from_string(&std::fs::read_to_string(path)?)
}

/// Read from string instead of file.
///
///
/// # Examples
///
/// ```
/// let s = r#"
///     <robot name="robot">
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
///    "#;
/// let urdf_robot = urdf_rs::read_from_string(s).unwrap();
/// println!("{:?}", urdf_robot.links[0].visual[0].origin.xyz);
/// ```
pub fn read_from_string(string: &str) -> Result<Robot> {
    let sorted_string = sort_link_joint(string)?;
    serde_xml_rs::from_str(&sorted_string).map_err(UrdfError::new)
}

pub fn write_to_string(robot: &Robot) -> Result<String> {
    let mut buffer = String::new();
    let mut s = quick_xml::se::Serializer::new(&mut buffer);
    s.indent(' ', 2);
    robot.serialize(s).map_err(UrdfError::new)?;
    Ok(buffer)
}

#[cfg(test)]
mod tests {
    use crate::{read_from_string, write_to_string};
    use crate::{Geometry, JointType, Robot};
    use assert_approx_eq::assert_approx_eq;

    fn check_robot(robot: &Robot) {
        assert_eq!(robot.name, "robot");

        // <link>
        assert_eq!(robot.links.len(), 3);
        let link = &robot.links[0];
        assert_eq!(link.name, "shoulder1");
        let xyz = link.inertial.origin.xyz;
        assert_approx_eq!(xyz[0], 0.0);
        assert_approx_eq!(xyz[1], 0.0);
        assert_approx_eq!(xyz[2], 0.5);
        let rpy = link.inertial.origin.rpy;
        assert_approx_eq!(rpy[0], 0.0);
        assert_approx_eq!(rpy[1], 0.0);
        assert_approx_eq!(rpy[2], 0.0);
        assert_approx_eq!(link.inertial.mass.value, 1.0);
        assert_approx_eq!(link.inertial.inertia.ixx, 100.0);
        assert_approx_eq!(link.inertial.inertia.ixy, 0.0);
        assert_approx_eq!(link.inertial.inertia.ixz, 0.0);
        assert_approx_eq!(link.inertial.inertia.iyy, 100.0);
        assert_approx_eq!(link.inertial.inertia.ixz, 0.0);
        assert_approx_eq!(link.inertial.inertia.izz, 100.0);

        assert_eq!(link.visual.len(), 3);
        let xyz = &link.visual[0].origin.xyz;
        assert_approx_eq!(xyz[0], 0.1);
        assert_approx_eq!(xyz[1], 0.2);
        assert_approx_eq!(xyz[2], 0.3);
        let rpy = &link.visual[0].origin.rpy;
        assert_approx_eq!(rpy[0], -0.1);
        assert_approx_eq!(rpy[1], -0.2);
        assert_approx_eq!(rpy[2], -0.3);

        // https://github.com/openrr/urdf-rs/issues/94
        let xyz = &link.visual[1].origin.xyz;
        assert_approx_eq!(xyz[0], 0.1);
        assert_approx_eq!(xyz[1], 0.2);
        assert_approx_eq!(xyz[2], 0.3);
        let rpy = &link.visual[1].origin.rpy;
        assert_approx_eq!(rpy[0], -0.1);
        assert_approx_eq!(rpy[1], -0.2);
        assert_approx_eq!(rpy[2], -0.3);

        let xyz = &link.visual[2].origin.xyz;
        assert_approx_eq!(xyz[0], 0.1);
        assert_approx_eq!(xyz[1], 0.2);
        assert_approx_eq!(xyz[2], 0.3);
        let rpy = &link.visual[2].origin.rpy;
        assert_approx_eq!(rpy[0], -0.1);
        assert_approx_eq!(rpy[1], -0.2);
        assert_approx_eq!(rpy[2], -0.3);

        // https://github.com/openrr/urdf-rs/issues/95
        assert!(link.visual[0].material.is_some());
        let mat = link.visual[0].material.as_ref().unwrap();
        assert_eq!(mat.name, "Cyan");
        let rgba = mat.color.clone().unwrap().rgba;
        assert_approx_eq!(rgba[0], 0.0);
        assert_approx_eq!(rgba[1], 1.0);
        assert_approx_eq!(rgba[2], 1.0);
        assert_approx_eq!(rgba[3], 1.0);

        match &link.visual[0].geometry {
            Geometry::Box { size } => {
                assert_approx_eq!(size[0], 1.0f64);
                assert_approx_eq!(size[1], 2.0f64);
                assert_approx_eq!(size[2], 3.0f64);
            }
            _ => panic!("geometry error"),
        }
        match &link.visual[1].geometry {
            Geometry::Mesh {
                ref filename,
                scale,
            } => {
                assert_eq!(filename, "aa.dae");
                assert!(scale.is_none());
            }
            _ => panic!("geometry error"),
        }
        match &link.visual[2].geometry {
            Geometry::Mesh {
                ref filename,
                scale,
            } => {
                assert_eq!(filename, "bbb.dae");
                let scale = scale.as_ref().unwrap();
                assert_approx_eq!(scale[0], 2.0);
                assert_approx_eq!(scale[1], 3.0);
                assert_approx_eq!(scale[2], 4.0);
            }
            _ => panic!("geometry error"),
        }

        assert_eq!(link.collision.len(), 1);
        let xyz = &link.collision[0].origin.xyz;
        assert_approx_eq!(xyz[0], 0.0);
        assert_approx_eq!(xyz[1], 0.0);
        assert_approx_eq!(xyz[2], 0.0);
        let rpy = &link.collision[0].origin.rpy;
        assert_approx_eq!(rpy[0], 0.0);
        assert_approx_eq!(rpy[1], 0.0);
        assert_approx_eq!(rpy[2], 0.0);
        match &link.collision[0].geometry {
            Geometry::Cylinder { radius, length } => {
                assert_approx_eq!(radius, 1.0);
                assert_approx_eq!(length, 0.5);
            }
            _ => panic!("geometry error"),
        }

        assert_eq!(robot.links[1].name, "elbow1");
        assert_eq!(robot.links[2].name, "wrist1");

        // <material>
        assert_eq!(robot.materials.len(), 1);
        let mat = &robot.materials[0];
        assert_eq!(mat.name, "blue");
        assert!(mat.color.is_some());
        let rgba = mat.color.clone().unwrap().rgba;
        assert_approx_eq!(rgba[0], 0.0);
        assert_approx_eq!(rgba[1], 0.0);
        assert_approx_eq!(rgba[2], 0.8);
        assert_approx_eq!(rgba[3], 1.0);

        // <joint>
        assert_eq!(robot.joints.len(), 2);
        let joint = &robot.joints[0];
        assert_eq!(joint.name, "shoulder_pitch");
        assert_eq!(joint.parent.link, "shoulder1");
        assert_eq!(joint.child.link, "elbow1");
        assert_eq!(joint.joint_type, JointType::Revolute);
        assert_approx_eq!(joint.limit.upper, 1.0);
        assert_approx_eq!(joint.limit.lower, -1.0);
        assert_approx_eq!(joint.limit.effort, 0.0);
        assert_approx_eq!(joint.limit.velocity, 1.0);
        assert_eq!(joint.calibration.as_ref().unwrap().rising, None);
        assert_eq!(joint.calibration.as_ref().unwrap().falling, None);
        assert_approx_eq!(joint.dynamics.as_ref().unwrap().damping, 0.0);
        assert_approx_eq!(joint.dynamics.as_ref().unwrap().friction, 0.0);
        assert_eq!(joint.mimic.as_ref().unwrap().joint, "elbow1");
        assert_approx_eq!(joint.safety_controller.as_ref().unwrap().k_velocity, 10.0);
        assert!(joint.mimic.as_ref().unwrap().multiplier.is_none());
        assert!(joint.mimic.as_ref().unwrap().offset.is_none());
        let xyz = &joint.axis.xyz;
        assert_approx_eq!(xyz[0], 0.0);
        assert_approx_eq!(xyz[1], 1.0);
        assert_approx_eq!(xyz[2], -1.0);

        let joint = &robot.joints[1];
        assert_eq!(joint.name, "shoulder_pitch");
        assert_eq!(joint.parent.link, "elbow1");
        assert_eq!(joint.child.link, "wrist1");
        assert_eq!(joint.joint_type, JointType::Revolute);
        assert_approx_eq!(joint.limit.upper, 1.0);
        assert_approx_eq!(joint.limit.lower, -2.0);
        assert_approx_eq!(joint.limit.effort, 0.0);
        assert_approx_eq!(joint.limit.velocity, 1.0);
        assert_approx_eq!(joint.dynamics.as_ref().unwrap().damping, 10.0);
        assert_approx_eq!(joint.dynamics.as_ref().unwrap().friction, 1.0);
        assert_eq!(joint.mimic.as_ref().unwrap().joint, "shoulder1");
        assert_approx_eq!(joint.mimic.as_ref().unwrap().multiplier.unwrap(), 5.0);
        assert_approx_eq!(joint.mimic.as_ref().unwrap().offset.unwrap(), 1.0);
        assert_approx_eq!(joint.safety_controller.as_ref().unwrap().k_position, 10.0);
        assert_approx_eq!(joint.safety_controller.as_ref().unwrap().k_velocity, 1.0);
        assert_approx_eq!(
            joint.safety_controller.as_ref().unwrap().soft_lower_limit,
            -0.5
        );
        assert_approx_eq!(
            joint.safety_controller.as_ref().unwrap().soft_upper_limit,
            1.0
        );
        let xyz = &joint.axis.xyz;
        assert_approx_eq!(xyz[0], 0.0);
        assert_approx_eq!(xyz[1], 1.0);
        assert_approx_eq!(xyz[2], 0.0);
    }

    #[test]
    fn deserialization() {
        let s = r#"
            <robot name="robot" xmlns="http://www.ros.org">
                <material name="blue">
                  <color rgba="0.0 0.0 0.8 1.0"/>
                </material>

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
                    <visual>
                        <geometry>
                            <mesh filename="aa.dae" />
                        </geometry>
                        <origin xyz="0.1 0.2 0.3" rpy="-0.1 -0.2  -0.3" />
                    </visual>
                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <cylinder radius="1" length="0.5"/>
                        </geometry>
                    </collision>
                    <visual>
                        <origin xyz="0.1 0.2 0.3" rpy="-0.1 -0.2  -0.3" />
                        <geometry>
                            <mesh filename="bbb.dae" scale="2.0 3.0 4.0" />
                        </geometry>
                    </visual>
                </link>
                <joint name="shoulder_pitch" type="revolute">
                    <origin xyz="0.0 0.0 0.1" />
                    <parent link="shoulder1" />
                    <child link="elbow1" />
                    <axis xyz="0 1 -1" />
                    <calibration />
                    <dynamics />
                    <mimic joint="elbow1" />
                    <safety_controller k_velocity="10" />
                    <limit lower="-1" upper="1.0" effort="0" velocity="1.0"/>
                </joint>
                <link name="elbow1" />
                <link name="wrist1" />
                <joint name="shoulder_pitch" type="revolute">
                    <origin xyz="0.0 0.0 0.0" />
                    <parent link="elbow1" />
                    <child link="wrist1" />
                    <axis xyz=" 0 1 0 " />
                    <calibration falling="1" rising="1" />
                    <dynamics damping="10.0" friction="1" />
                    <mimic joint="shoulder1" offset="1" multiplier="5" />
                    <safety_controller k_position="10" k_velocity="1" soft_lower_limit="-0.5" soft_upper_limit="1" />
                    <limit lower=" -2" upper="1.0 " effort="0 " velocity=" 1.0 "/>
                </joint>
            </robot>
        "#;
        let robot = read_from_string(s).unwrap();
        check_robot(&robot);

        // Loopback test
        let s = write_to_string(&robot).unwrap();
        assert!(!s.contains("Robot"), "{s}"); // https://github.com/openrr/urdf-rs/issues/80
        let robot = read_from_string(&s).unwrap();
        check_robot(&robot);
    }

    fn quat_robot(version: &str, origin_attrs: &str) -> String {
        format!(
            r#"
            <robot name="robot" {version}>
                <link name="a">
                    <visual>
                        <origin xyz="1 2 3" {origin_attrs}/>
                        <geometry><sphere radius="1"/></geometry>
                    </visual>
                </link>
                <link name="b" />
                <joint name="j" type="fixed">
                    <origin {origin_attrs}/>
                    <parent link="a" />
                    <child link="b" />
                </joint>
            </robot>
            "#
        )
    }

    fn assert_rpy(robot: &Robot, expected: [f64; 3]) {
        let poses = [&robot.links[0].visual[0].origin, &robot.joints[0].origin];
        for pose in poses {
            for (actual, expected) in pose.rpy.iter().zip(expected) {
                assert_approx_eq!(*actual, expected);
            }
        }
        assert_eq!(*robot.links[0].visual[0].origin.xyz, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn quat_xyzw_version_1_1() {
        use std::f64::consts::FRAC_PI_2;
        let cases = [
            ("0.5 0.5 0.5 0.5", [FRAC_PI_2, 0.0, FRAC_PI_2]),
            ("0 0 0 1", [0.0, 0.0, 0.0]),
            // not normalized
            ("0 0 0 2", [0.0, 0.0, 0.0]),
            ("0 0 0 0", [0.0, 0.0, 0.0]),
            // gimbal lock
            ("0 1 0 1", [0.0, FRAC_PI_2, 0.0]),
            ("0 -1 0 1", [0.0, -FRAC_PI_2, 0.0]),
            (" 0 0 1 0 ", [0.0, 0.0, std::f64::consts::PI]),
        ];
        for version in [r#"version="1.1""#, r#"version="1.2""#] {
            for (quat, expected) in cases {
                let s = quat_robot(version, &format!(r#"quat_xyzw="{quat}""#));
                let robot = read_from_string(&s).unwrap();
                assert_rpy(&robot, expected);

                // Loopback test
                let robot = read_from_string(&write_to_string(&robot).unwrap()).unwrap();
                assert_rpy(&robot, expected);
            }
        }
    }

    #[test]
    fn quat_xyzw_ignored_before_version_1_1() {
        for version in ["", r#"version="1.0""#] {
            let s = quat_robot(version, r#"quat_xyzw="0.5 0.5 0.5 0.5""#);
            let robot = read_from_string(&s).unwrap();
            assert_rpy(&robot, [0.0, 0.0, 0.0]);

            let s = quat_robot(version, r#"rpy="0.1 0.2 0.3" quat_xyzw="0.5 0.5 0.5 0.5""#);
            let robot = read_from_string(&s).unwrap();
            assert_rpy(&robot, [0.1, 0.2, 0.3]);
        }
    }

    #[test]
    fn quat_xyzw_errors() {
        let version = r#"version="1.1""#;
        let s = quat_robot(version, r#"rpy="0 0 0" quat_xyzw="0 0 0 1""#);
        assert!(read_from_string(&s).is_err());
        for quat in ["0 0 1", "0 0 0 1 0", "0 0 a 1", ""] {
            let s = quat_robot(version, &format!(r#"quat_xyzw="{quat}""#));
            assert!(read_from_string(&s).is_err(), "{quat}");
        }
    }

    #[test]
    fn quat_xyzw_all_origins() {
        // <inertial>, <visual>, <collision> and <joint> origins, with xmlns
        let s = r#"
            <robot name="robot" version="1.1" xmlns="http://www.ros.org">
                <link name="a">
                    <inertial>
                        <origin xyz="0 0 0.5" quat_xyzw="0.5 0.5 0.5 0.5"/>
                        <mass value="1"/>
                        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
                    </inertial>
                    <visual>
                        <origin quat_xyzw="0.5 0.5 0.5 0.5"/>
                        <geometry><sphere radius="1"/></geometry>
                    </visual>
                    <collision>
                        <origin quat_xyzw="0.5 0.5 0.5 0.5"/>
                        <geometry><sphere radius="1"/></geometry>
                    </collision>
                    <collision>
                        <origin rpy="0.1 0.2 0.3"/>
                        <geometry><sphere radius="1"/></geometry>
                    </collision>
                </link>
                <link name="b" />
                <joint name="j" type="revolute">
                    <origin quat_xyzw="0.5 0.5 0.5 0.5"/>
                    <parent link="a" />
                    <child link="b" />
                    <axis xyz="0 0 1" />
                    <limit lower="-1" upper="1" effort="0" velocity="1"/>
                </joint>
            </robot>
        "#;
        use std::f64::consts::FRAC_PI_2;
        let robot = read_from_string(s).unwrap();
        let link = &robot.links[0];
        let poses = [
            &link.inertial.origin,
            &link.visual[0].origin,
            &link.collision[0].origin,
            &robot.joints[0].origin,
        ];
        for pose in poses {
            for (actual, expected) in pose.rpy.iter().zip([FRAC_PI_2, 0.0, FRAC_PI_2]) {
                assert_approx_eq!(*actual, expected);
            }
        }
        assert_eq!(*link.inertial.origin.xyz, [0.0, 0.0, 0.5]);
        // origins without quat_xyzw are not affected
        assert_eq!(*link.collision[1].origin.rpy, [0.1, 0.2, 0.3]);
    }

    #[test]
    fn urdf_version() {
        let cases = [
            ("", (1, 0)),
            (r#"version="1.0""#, (1, 0)),
            (r#"version="1.1""#, (1, 1)),
            (r#"version=" 1.2 ""#, (1, 2)),
            (r#"version="2.0""#, (2, 0)),
            // compared as numbers, not strings
            (r#"version="1.10""#, (1, 10)),
            // malformed version is treated as 1.0
            (r#"version="1""#, (1, 0)),
            (r#"version="1.1.0""#, (1, 0)),
            (r#"version="a.b""#, (1, 0)),
            (r#"version="""#, (1, 0)),
        ];
        assert!(super::UrdfVersion::new(1, 10) > super::UrdfVersion::V1_2);
        for (version, expected) in cases {
            let e: xml::Element = format!("<robot {version}/>").parse().unwrap();
            let expected = super::UrdfVersion::new(expected.0, expected.1);
            assert_eq!(super::UrdfVersion::from_robot(&e), expected, "{version}");

            // quat_xyzw is used only for 1.1 or later
            let s = quat_robot(version, r#"quat_xyzw="0.5 0.5 0.5 0.5""#);
            let robot = read_from_string(&s).unwrap();
            let rpy = if expected >= super::UrdfVersion::V1_1 {
                [
                    std::f64::consts::FRAC_PI_2,
                    0.0,
                    std::f64::consts::FRAC_PI_2,
                ]
            } else {
                [0.0; 3]
            };
            assert_rpy(&robot, rpy);
        }
    }

    /// Same as `urdf::Rotation::setFromRPY` of urdfdom_headers.
    fn rpy_to_quaternion([r, p, y]: [f64; 3]) -> [f64; 4] {
        let (sr, cr) = (r / 2.0).sin_cos();
        let (sp, cp) = (p / 2.0).sin_cos();
        let (sy, cy) = (y / 2.0).sin_cos();
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    }

    #[test]
    fn quaternion_to_rpy_round_trip() {
        let angles = [-3.0, -2.0, -1.5, -0.7, -0.1, 0.0, 0.1, 0.7, 1.5, 2.0, 3.0];
        let pitches = [-1.5, -0.7, -0.1, 0.0, 0.1, 0.7, 1.5];
        for r in angles {
            for p in pitches {
                for y in angles {
                    let q = rpy_to_quaternion([r, p, y]);
                    let rpy = super::quaternion_to_rpy(q);
                    // pitch within (-pi/2, pi/2) gives the unique rpy
                    assert_approx_eq!(rpy[0], r, 1e-9);
                    assert_approx_eq!(rpy[1], p, 1e-9);
                    assert_approx_eq!(rpy[2], y, 1e-9);

                    // the scale and the sign of the quaternion do not matter
                    let q2 = q.map(|v| v * -3.0);
                    let rpy2 = super::quaternion_to_rpy(q2);
                    for (a, b) in rpy.iter().zip(rpy2) {
                        assert_approx_eq!(*a, b, 1e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn quaternion_to_rpy_gimbal_lock() {
        use std::f64::consts::FRAC_PI_2;
        // When pitch is +-pi/2, only roll - yaw (or roll + yaw) is determined,
        // so compare the rotations instead of the angles.
        for p in [FRAC_PI_2, -FRAC_PI_2] {
            for (r, y) in [(0.0, 0.0), (0.3, 0.0), (0.0, 0.3), (0.5, -1.0), (1.0, 2.0)] {
                let q = rpy_to_quaternion([r, p, y]);
                let rpy = super::quaternion_to_rpy(q);
                assert_approx_eq!(rpy[0], 0.0);
                assert_approx_eq!(rpy[1], p);
                let q2 = rpy_to_quaternion(rpy);
                let dot: f64 = q.iter().zip(q2).map(|(a, b)| a * b).sum();
                assert_approx_eq!(dot.abs(), 1.0, 1e-9);
            }
        }
    }

    fn limit_robot(version: &str, limit_attrs: &str) -> String {
        format!(
            r#"
            <robot name="robot" {version}>
                <link name="a" />
                <link name="b" />
                <joint name="j" type="revolute">
                    <parent link="a" />
                    <child link="b" />
                    <limit {limit_attrs}/>
                </joint>
            </robot>
            "#
        )
    }

    fn read_limit(version: &str, limit_attrs: &str) -> crate::Result<crate::JointLimit> {
        let robot = read_from_string(&limit_robot(version, limit_attrs))?;
        Ok(robot.joints[0].limit.clone())
    }

    fn assert_loopback(robot: &Robot) {
        let s = write_to_string(robot).unwrap();
        assert!(!s.contains("inf"), "{s}");
        let robot2 = read_from_string(&s).unwrap();
        assert_eq!(robot.version, robot2.version);
        for (j1, j2) in robot.joints.iter().zip(&robot2.joints) {
            assert_eq!(j1.limit, j2.limit, "{s}");
        }
    }

    #[test]
    fn joint_limit_version_1_2() {
        let inf = f64::INFINITY;
        let v = r#"version="1.2""#;

        let attrs = r#"lower="-1" upper="2" effort="3" velocity="4" acceleration="5" deceleration="6" jerk="7""#;
        let limit = read_limit(v, attrs).unwrap();
        assert_eq!(
            limit,
            crate::JointLimit {
                lower: -1.0,
                upper: 2.0,
                effort: 3.0,
                velocity: 4.0,
                acceleration: 5.0,
                deceleration: 6.0,
                jerk: 7.0,
            }
        );

        // omitted limits mean no limit, even for revolute joints
        let limit = read_limit(v, "").unwrap();
        assert_eq!(
            limit,
            crate::JointLimit {
                lower: -inf,
                upper: inf,
                effort: inf,
                velocity: inf,
                acceleration: inf,
                deceleration: inf,
                jerk: inf,
            }
        );

        // deceleration defaults to acceleration
        let limit = read_limit(v, r#"acceleration=" 5 ""#).unwrap();
        assert_eq!(limit.acceleration, 5.0);
        assert_eq!(limit.deceleration, 5.0);
        let limit = read_limit(v, r#"deceleration="6""#).unwrap();
        assert_eq!(limit.acceleration, inf);
        assert_eq!(limit.deceleration, 6.0);

        // zero limits and lower == upper are valid
        let attrs = r#"lower="1" upper="1" effort="0" velocity="0" acceleration="0" deceleration="0" jerk="0""#;
        let limit = read_limit(v, attrs).unwrap();
        assert_eq!((limit.lower, limit.upper, limit.jerk), (1.0, 1.0, 0.0));

        // later versions also use the 1.2 semantics
        let limit = read_limit(r#"version="1.3""#, "").unwrap();
        assert_eq!(limit.velocity, inf);

        // Loopback test
        for attrs in [
            "",
            r#"lower="-1" upper="2" effort="3" velocity="4" acceleration="5" deceleration="6" jerk="7""#,
            r#"acceleration="5""#,
            r#"deceleration="6""#,
            r#"lower="-1""#,
        ] {
            let robot = read_from_string(&limit_robot(v, attrs)).unwrap();
            assert_loopback(&robot);
        }
    }

    #[test]
    fn joint_limit_infinite_deceleration_loopback() {
        // deceleration is infinity while acceleration is finite: this cannot
        // be expressed by omitting deceleration, so "inf" is written.
        let mut robot = read_from_string(&limit_robot(r#"version="1.2""#, "")).unwrap();
        robot.joints[0].limit.acceleration = 5.0;
        let s = write_to_string(&robot).unwrap();
        assert!(s.contains(r#"deceleration="inf""#), "{s}");
        let robot2 = read_from_string(&s).unwrap();
        assert_eq!(robot.joints[0].limit, robot2.joints[0].limit);
    }

    #[test]
    fn joint_limit_before_version_1_2() {
        let inf = f64::INFINITY;
        for v in ["", r#"version="1.0""#, r#"version="1.1""#] {
            // acceleration, deceleration and jerk are ignored
            let attrs = r#"lower="-1" upper="2" effort="3" velocity="4" acceleration="5" deceleration="6" jerk="7""#;
            let limit = read_limit(v, attrs).unwrap();
            assert_eq!(
                limit,
                crate::JointLimit {
                    lower: -1.0,
                    upper: 2.0,
                    effort: 3.0,
                    velocity: 4.0,
                    acceleration: inf,
                    deceleration: inf,
                    jerk: inf,
                }
            );

            // omitted lower, upper and effort are 0
            let limit = read_limit(v, r#"velocity="4""#).unwrap();
            assert_eq!((limit.lower, limit.upper, limit.effort), (0.0, 0.0, 0.0));

            // velocity is required
            assert!(read_limit(v, "").is_err());

            // values are not checked
            let attrs = r#"lower="2" upper="-1" effort="-3" velocity="-4""#;
            assert!(read_limit(v, attrs).is_ok());
            // invalid values of the ignored attributes are not errors
            let attrs = r#"velocity="4" acceleration="a" deceleration="-1" jerk="NaN""#;
            assert!(read_limit(v, attrs).is_ok());

            let robot = read_from_string(&limit_robot(v, r#"velocity="4""#)).unwrap();
            assert_loopback(&robot);
        }
    }

    #[test]
    fn joint_limit_errors_version_1_2() {
        let v = r#"version="1.2""#;
        for attrs in [
            r#"lower="1" upper="0""#,
            r#"effort="-1""#,
            r#"velocity="-1""#,
            r#"acceleration="-1""#,
            r#"deceleration="-1""#,
            r#"jerk="-1""#,
            r#"lower="a""#,
            r#"effort="""#,
            r#"lower="NaN""#,
            r#"upper="NaN""#,
            r#"effort="NaN""#,
            r#"velocity="NaN""#,
            r#"acceleration="NaN""#,
            r#"deceleration="NaN""#,
            r#"jerk="NaN""#,
        ] {
            assert!(read_limit(v, attrs).is_err(), "{attrs}");
        }

        // the error message contains the joint name
        let err = read_limit(v, r#"effort="-1""#).unwrap_err();
        assert!(err.to_string().contains("joint [j]"), "{err}");
    }

    #[test]
    fn joint_limit_other_joint_types_version_1_2() {
        let inf = f64::INFINITY;
        for joint_type in ["prismatic", "continuous", "fixed", "floating", "planar"] {
            let s = format!(
                r#"
                <robot name="robot" version="1.2">
                    <link name="a" />
                    <link name="b" />
                    <link name="c" />
                    <joint name="j1" type="{joint_type}">
                        <parent link="a" />
                        <child link="b" />
                        <limit />
                    </joint>
                    <joint name="j2" type="{joint_type}">
                        <parent link="b" />
                        <child link="c" />
                    </joint>
                </robot>
                "#
            );
            let robot = read_from_string(&s).unwrap();
            // omitted attributes of <limit> mean no limit
            let limit = &robot.joints[0].limit;
            assert_eq!(
                (limit.lower, limit.upper, limit.effort, limit.velocity),
                (-inf, inf, inf, inf),
                "{joint_type}"
            );
            // without <limit>, the default is used as before
            assert_eq!(robot.joints[1].limit, crate::JointLimit::default());
            assert_loopback(&robot);
        }
    }

    fn geometry_robot(version: &str, geometry: &str) -> String {
        format!(
            r#"
            <robot name="robot" {version}>
                <link name="a">
                    <visual>
                        <geometry>{geometry}</geometry>
                    </visual>
                    <collision>
                        <geometry>{geometry}</geometry>
                    </collision>
                </link>
            </robot>
            "#
        )
    }

    #[test]
    fn geometry_version_1_2() {
        let valid = [
            r#"<sphere radius="1"/>"#,
            r#"<box size="1 2 3"/>"#,
            r#"<cylinder radius="1" length="2"/>"#,
            r#"<capsule radius="1" length="2"/>"#,
            r#"<mesh filename="a.stl" scale="0 -1 1"/>"#,
        ];
        let invalid = [
            r#"<sphere radius="0"/>"#,
            r#"<sphere radius="-1"/>"#,
            r#"<sphere radius="inf"/>"#,
            r#"<sphere radius="NaN"/>"#,
            r#"<box size="1 0 3"/>"#,
            r#"<box size="1 2 -3"/>"#,
            r#"<cylinder radius="0" length="2"/>"#,
            r#"<cylinder radius="1" length="-2"/>"#,
            r#"<capsule radius="-1" length="2"/>"#,
            r#"<capsule radius="1" length="0"/>"#,
        ];
        for g in valid {
            for v in ["", r#"version="1.1""#, r#"version="1.2""#] {
                assert!(read_from_string(&geometry_robot(v, g)).is_ok(), "{v} {g}");
            }
        }
        for g in invalid {
            // not checked before 1.2
            for v in ["", r#"version="1.1""#] {
                assert!(read_from_string(&geometry_robot(v, g)).is_ok(), "{v} {g}");
            }
            let v = r#"version="1.2""#;
            assert!(read_from_string(&geometry_robot(v, g)).is_err(), "{v} {g}");
        }
    }

    #[test]
    fn robot_version() {
        let robot = read_from_string(&limit_robot("", r#"velocity="1""#)).unwrap();
        assert_eq!(robot.version, None);
        let s = write_to_string(&robot).unwrap();
        assert!(!s.contains("version"), "{s}");

        for v in ["1.0", "1.1", "1.2"] {
            let robot = read_from_string(&limit_robot(
                &format!(r#"version="{v}""#),
                r#"velocity="1""#,
            ))
            .unwrap();
            assert_eq!(robot.version.as_deref(), Some(v));
            let s = write_to_string(&robot).unwrap();
            assert!(s.contains(&format!(r#"version="{v}""#)), "{s}");
        }
    }
}
