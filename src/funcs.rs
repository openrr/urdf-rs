use deserialize::*;
use errors::*;

use serde_xml_rs;
use std::path::Path;
use std::io::prelude::*;
use xml;
use std::fs::File;

/// sort <link> and <joint> to avoid the [issue](https://github.com/RReverser/serde-xml-rs/issues/5)
fn sort_link_joint(string: &str) -> Result<String> {
    let e: xml::Element = string.parse()?;
    let mut links = Vec::new();
    let mut joints = Vec::new();
    let mut materials = Vec::new();
    for c in &e.children {
        if let xml::Xml::ElementNode(ref xml_elm) = *c {
            if xml_elm.name == "link" {
                links.push(xml::Xml::ElementNode(xml_elm.clone()));
            } else if xml_elm.name == "joint" {
                joints.push(xml::Xml::ElementNode(xml_elm.clone()));
            } else if xml_elm.name == "material" {
                materials.push(xml::Xml::ElementNode(xml_elm.clone()));
            }
        };
    }
    let mut new_elm = e.clone();
    links.extend(joints);
    links.extend(materials);
    new_elm.children = links;
    Ok(format!("{}", new_elm))
}

/// Read urdf file and create Robot instance
///
/// # Examples
///
/// ```
/// extern crate urdf_rs;
/// let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
/// let links = urdf_robo.links;
/// println!("{:?}", links[0].visual[0].origin.xyz);
/// ```
pub fn read_file<P: AsRef<Path>>(path: P) -> Result<Robot> {
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
/// let urdf_robo = urdf_rs::read_from_string(s).unwrap();
/// println!("{:?}", urdf_robo.links[0].visual[0].origin.xyz);
/// ```

pub fn read_from_string(string: &str) -> Result<Robot> {
    let sorted_string = sort_link_joint(string)?;
    serde_xml_rs::from_str(&sorted_string).map_err(From::from)
}

#[test]
fn it_works() {
    let s = r##"
        <robot name="robo">
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
                <collision>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <geometry>
                        <cylinder radius="1" length="0.5"/>
                    </geometry>
                </collision>
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
    let robo = read_from_string(s).unwrap();

    assert_eq!(robo.name, "robo");
    assert_eq!(robo.links.len(), 3);
    assert_eq!(robo.joints.len(), 2);
    assert_eq!(robo.links[0].visual.len(), 1);
    let xyz = robo.links[0].visual[0].origin.xyz;
    assert_eq!(xyz[0], 0.1);
    assert_eq!(xyz[1], 0.2);
    assert_eq!(xyz[2], 0.3);
    let rpy = robo.links[0].visual[0].origin.rpy;
    assert_eq!(rpy[0], -0.1);
    assert_eq!(rpy[1], -0.2);
    assert_eq!(rpy[2], -0.3);

    match robo.links[0].visual[0].geometry {
        Geometry::Box { size } => {
            assert_eq!(size[0], 1.0f64);
            assert_eq!(size[1], 2.0f64);
            assert_eq!(size[2], 3.0f64);
        }
        _ => panic!("geometry error"),
    }
    assert_eq!(robo.materials.len(), 1);

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
