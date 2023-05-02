use crate::deserialize::*;
use crate::errors::*;

use std::path::Path;

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
    yaserde::de::from_str(&string).map_err(UrdfError::new)
}

pub fn write_to_string(robot: &Robot) -> Result<String> {
    let conf = yaserde::ser::Config {
        perform_indent: true,
        write_document_declaration: false,
        indent_string: None,
    };
    yaserde::ser::to_string_with_config(robot, &conf).map_err(UrdfError::new)
}

#[cfg(test)]
mod tests {
    use crate::{read_from_string, write_to_string};
    use crate::{BoxGeometry, CylinderGeometry, Geometry, MeshGeometry, Robot};
    use assert_approx_eq::assert_approx_eq;

    fn check_robot(robot: &Robot) {
        assert_eq!(robot.name, "robot");
        assert_eq!(robot.links.len(), 3);
        assert_eq!(robot.joints.len(), 2);
        assert_eq!(robot.links[0].visual.len(), 3);
        assert_eq!(robot.links[0].inertial.mass.value, 1.0);
        let xyz = &robot.links[0].visual[0].origin.xyz;
        assert_approx_eq!(xyz[0], 0.1);
        assert_approx_eq!(xyz[1], 0.2);
        assert_approx_eq!(xyz[2], 0.3);
        let rpy = &robot.links[0].visual[0].origin.rpy;
        assert_approx_eq!(rpy[0], -0.1);
        assert_approx_eq!(rpy[1], -0.2);
        assert_approx_eq!(rpy[2], -0.3);

        match (&robot.links[0].visual[0].geometry).into() {
            Geometry::Box(BoxGeometry { size }) => {
                assert_approx_eq!(size[0], 1.0f64);
                assert_approx_eq!(size[1], 2.0f64);
                assert_approx_eq!(size[2], 3.0f64);
            }
            _ => panic!("geometry error"),
        }
        match (&robot.links[0].visual[1].geometry).into() {
            Geometry::Mesh(MeshGeometry {
                ref filename,
                scale,
            }) => {
                assert_eq!(filename, "aa.dae");
                assert!(scale.is_none());
            }
            _ => panic!("geometry error"),
        }
        match (&robot.links[0].visual[2].geometry).into() {
            Geometry::Mesh(MeshGeometry {
                ref filename,
                scale,
            }) => {
                assert_eq!(filename, "bbb.dae");
                assert!(scale.is_some());
            }
            _ => panic!("geometry error"),
        }

        assert_eq!(robot.links[0].collision.len(), 1);
        match (&robot.links[0].collision[0].geometry).into() {
            Geometry::Cylinder(CylinderGeometry { radius, length }) => {
                assert_approx_eq!(radius, 1.0);
                assert_approx_eq!(length, 0.5);
            }
            _ => panic!("geometry error"),
        }

        assert_eq!(robot.materials.len(), 1);
        let mat = &robot.materials[0];
        assert_eq!(mat.name, "blue");
        assert!(mat.color.is_some());
        let rgba = mat.color.clone().unwrap().rgba;
        assert_approx_eq!(rgba[0], 0.0);
        assert_approx_eq!(rgba[1], 0.0);
        assert_approx_eq!(rgba[2], 0.8);
        assert_approx_eq!(rgba[3], 1.0);

        assert_eq!(robot.joints[0].name, "shoulder_pitch");
        assert_eq!(robot.joints[0].parent.link, "shoulder1");
        assert_eq!(robot.joints[0].child.link, "elbow1");
        let xyz = &robot.joints[0].axis.xyz;
        assert_approx_eq!(xyz[0], 0.0f64);
        assert_approx_eq!(xyz[1], 1.0f64);
        assert_approx_eq!(xyz[2], -1.0f64);
        let xyz = &robot.joints[0].axis.xyz;
        //"0 1 -1"
        assert_approx_eq!(xyz[0], 0.0);
        assert_approx_eq!(xyz[1], 1.0);
        assert_approx_eq!(xyz[2], -1.0);
    }

    #[test]
    fn deserialization() {
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
        dbg!(&robot);

        //check_robot(&robot);

        // Loopback test
        let s = write_to_string(&robot).unwrap();
        println!("{}", s);

        let robot = read_from_string(&s).unwrap();
        //dbg!(&robot);
        check_robot(&robot);
    }
}
