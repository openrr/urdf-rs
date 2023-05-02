use crate::deserialize::*;
use crate::errors::*;

use std::path::Path;

/// sort <link> and <joint> to avoid the [issue](https://github.com/RReverser/serde-xml-rs/issues/5)
fn sort_link_joint(string: &str) -> Result<String> {
    let e: xml::Element = string.parse().map_err(UrdfError::new)?;
    let mut links = Vec::new();
    let mut joints = Vec::new();
    let mut materials = Vec::new();
    for c in &e.children {
        if let xml::Xml::ElementNode(xml_elm) = c {
            if xml_elm.name == "link" {
                links.push(sort_visual_collision(xml_elm));
            } else if xml_elm.name == "joint" {
                joints.push(xml::Xml::ElementNode(xml_elm.clone()));
            } else if xml_elm.name == "material" {
                materials.push(xml::Xml::ElementNode(xml_elm.clone()));
            }
        }
    }
    let mut new_elm = e;
    links.extend(joints);
    links.extend(materials);
    new_elm.children = links;
    Ok(format!("{new_elm}"))
}

fn sort_visual_collision(elm: &xml::Element) -> xml::Xml {
    let mut visuals = Vec::new();
    let mut collisions = Vec::new();
    for c in &elm.children {
        if let xml::Xml::ElementNode(xml_elm) = c {
            if xml_elm.name == "visual" || xml_elm.name == "inertial" {
                visuals.push(xml::Xml::ElementNode(xml_elm.clone()));
            } else if xml_elm.name == "collision" {
                collisions.push(xml::Xml::ElementNode(xml_elm.clone()));
            }
        }
    }
    let mut new_elm = elm.clone();
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
/// let s = r##"
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
///    "##;
/// let urdf_robot = urdf_rs::read_from_string(s).unwrap();
/// println!("{:?}", urdf_robot.links[0].visual[0].origin.xyz);
/// ```

pub fn read_from_string(string: &str) -> Result<Robot> {
    let sorted_string = sort_link_joint(string)?;
    serde_xml_rs::from_str(&sorted_string).map_err(UrdfError::new)
}

#[test]
fn it_works() {
    use assert_approx_eq::assert_approx_eq;

    let s = r##"
        <robot name="robot">
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
                    <origin xyz="0.1 0.2 0.3" rpy="-0.1 -0.2  -0.3" />
                    <geometry>
                        <mesh filename="aa.dae" />
                    </geometry>
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
                <limit lower="-1" upper="1.0" effort="0" velocity="1.0"/>
            </joint>
            <link name="elbow1" />
            <link name="wrist1" />
            <joint name="shoulder_pitch" type="revolute">
                <origin xyz="0.0 0.0 0.0" />
                <parent link="elbow1" />
                <child link="wrist1" />
                <axis xyz="0 1 0" />
                <limit lower="-2" upper="1.0" effort="0" velocity="1.0"/>
            </joint>
        </robot>
    "##;
    let robot = read_from_string(s).unwrap();

    assert_eq!(robot.name, "robot");

    // materials
    assert_eq!(robot.materials.len(), 1);
    let mat = &robot.materials[0];
    assert_eq!(mat.name, "blue");
    let rgba = mat.color.clone().unwrap().rgba;
    assert_approx_eq!(rgba[0], 0.0);
    assert_approx_eq!(rgba[1], 0.0);
    assert_approx_eq!(rgba[2], 0.8);
    assert_approx_eq!(rgba[3], 1.0);

    // links
    assert_eq!(robot.links.len(), 3);
    assert_eq!(robot.links[0].name, "shoulder1");
    assert_eq!(robot.links[1].name, "elbow1");
    assert_eq!(robot.links[2].name, "wrist1");
    let xyz = robot.links[0].inertial.origin.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 0.0);
    assert_approx_eq!(xyz[2], 0.5);
    let rpy = robot.links[0].inertial.origin.rpy;
    assert_approx_eq!(rpy[0], 0.0);
    assert_approx_eq!(rpy[1], 0.0);
    assert_approx_eq!(rpy[2], 0.0);
    assert_approx_eq!(robot.links[0].inertial.mass.value, 1.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.ixx, 100.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.ixy, 0.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.ixz, 0.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.iyy, 100.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.iyz, 0.0);
    assert_approx_eq!(robot.links[0].inertial.inertia.izz, 100.0);

    assert_eq!(robot.links[0].visual.len(), 3);
    assert_eq!(robot.links[1].visual.len(), 0);
    assert_eq!(robot.links[2].visual.len(), 0);
    for visual in &robot.links[0].visual {
        let xyz = visual.origin.xyz;
        assert_approx_eq!(xyz[0], 0.1);
        assert_approx_eq!(xyz[1], 0.2);
        assert_approx_eq!(xyz[2], 0.3);
        let rpy = visual.origin.rpy;
        assert_approx_eq!(rpy[0], -0.1);
        assert_approx_eq!(rpy[1], -0.2);
        assert_approx_eq!(rpy[2], -0.3);
    }

    assert!(robot.links[0].visual[0].material.is_some());
    match robot.links[0].visual[0].geometry {
        Geometry::Box { size } => {
            assert_approx_eq!(size[0], 1.0f64);
            assert_approx_eq!(size[1], 2.0f64);
            assert_approx_eq!(size[2], 3.0f64);
        }
        _ => panic!("geometry error"),
    }
    assert!(robot.links[0].visual[1].material.is_none());
    match robot.links[0].visual[1].geometry {
        Geometry::Mesh {
            ref filename,
            scale,
        } => {
            assert_eq!(filename, "aa.dae");
            assert_eq!(scale, None);
        }
        _ => panic!("geometry error"),
    }
    assert!(robot.links[0].visual[2].material.is_none());
    match robot.links[0].visual[2].geometry {
        Geometry::Mesh {
            ref filename,
            scale,
        } => {
            assert_eq!(filename, "bbb.dae");
            assert_approx_eq!(scale.unwrap()[0], 2.0);
            assert_approx_eq!(scale.unwrap()[1], 3.0);
            assert_approx_eq!(scale.unwrap()[2], 4.0);
        }
        _ => panic!("geometry error"),
    }

    assert_eq!(robot.links[0].collision.len(), 1);
    let xyz = robot.links[0].collision[0].origin.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 0.0);
    assert_approx_eq!(xyz[2], 0.0);
    let rpy = robot.links[0].collision[0].origin.rpy;
    assert_approx_eq!(rpy[0], 0.0);
    assert_approx_eq!(rpy[1], 0.0);
    assert_approx_eq!(rpy[2], 0.0);
    match robot.links[0].collision[0].geometry {
        Geometry::Cylinder { radius, length } => {
            assert_approx_eq!(radius, 1.0);
            assert_approx_eq!(length, 0.5);
        }
        _ => panic!("geometry error"),
    }

    // joints
    assert_eq!(robot.joints.len(), 2);
    assert_eq!(robot.joints[0].name, "shoulder_pitch");
    assert_eq!(robot.joints[0].parent.link, "shoulder1");
    assert_eq!(robot.joints[0].child.link, "elbow1");
    assert_eq!(robot.joints[0].joint_type, JointType::Revolute);
    let xyz = robot.joints[0].origin.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 0.0);
    assert_approx_eq!(xyz[2], 0.1);
    let xyz = robot.joints[0].axis.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 1.0);
    assert_approx_eq!(xyz[2], -1.0);
    let limit = &robot.joints[0].limit;
    assert_approx_eq!(limit.lower, -1.0);
    assert_approx_eq!(limit.upper, 1.0);
    assert_approx_eq!(limit.effort, 0.0);
    assert_approx_eq!(limit.velocity, 1.0);

    assert_eq!(robot.joints[1].name, "shoulder_pitch");
    assert_eq!(robot.joints[1].parent.link, "elbow1");
    assert_eq!(robot.joints[1].child.link, "wrist1");
    assert_eq!(robot.joints[1].joint_type, JointType::Revolute);
    let xyz = robot.joints[1].origin.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 0.0);
    assert_approx_eq!(xyz[2], 0.0);
    let xyz = robot.joints[1].axis.xyz;
    assert_approx_eq!(xyz[0], 0.0);
    assert_approx_eq!(xyz[1], 1.0);
    assert_approx_eq!(xyz[2], 0.0);
    let limit = &robot.joints[1].limit;
    assert_approx_eq!(limit.lower, -2.0);
    assert_approx_eq!(limit.upper, 1.0);
    assert_approx_eq!(limit.effort, 0.0);
    assert_approx_eq!(limit.velocity, 1.0);
}
