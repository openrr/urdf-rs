use crate::deserialize::*;
use crate::errors::*;
use serde::Serialize;

use std::mem;
use std::path::Path;

/// sort <link> and <joint> to avoid the [issue](https://github.com/RReverser/serde-xml-rs/issues/5)
fn sort_link_joint(string: &str) -> Result<String> {
    let mut e: xml::Element = string.parse().map_err(UrdfError::new)?;
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
}
