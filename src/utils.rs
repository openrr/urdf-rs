use crate::deserialize::Robot;
use crate::errors::*;
use crate::funcs::*;

use once_cell::sync::Lazy;
use regex::Regex;
use std::borrow::Cow;
use std::path::Path;
use std::process::Command;

pub fn convert_xacro_to_urdf_with_args<P>(filename: P, args: &[(String, String)]) -> Result<String>
where
    P: AsRef<Path>,
{
    let output = Command::new("rosrun")
        .args(["xacro", "xacro", "--inorder"])
        .arg(filename.as_ref())
        .args(args.iter().map(|(k, v)| format!("{}:={}", k, v)))
        .output()
        .or_else(|_| {
            Command::new("xacro")
                .arg(filename.as_ref())
                .args(args.iter().map(|(k, v)| format!("{}:={}", k, v)))
                .output()
        })
        .expect("failed to execute xacro. install by apt-get install ros-*-xacro");
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?)
    } else {
        Err(ErrorKind::Command {
            msg: "failed to xacro".to_owned(),
            stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        }
        .into())
    }
}

pub fn convert_xacro_to_urdf<P>(filename: P) -> Result<String>
where
    P: AsRef<Path>,
{
    convert_xacro_to_urdf_with_args(filename, &[])
}

pub fn rospack_find(package: &str) -> Option<String> {
    let output = Command::new("rospack")
        .arg("find")
        .arg(package)
        .output()
        // support ROS2
        .or_else(|_| {
            Command::new("ros2")
                .args(["pkg", "prefix", "--share"])
                .arg(package)
                .output()
        })
        .expect("rospack find failed");
    if output.status.success() {
        String::from_utf8(output.stdout)
            .map(|string| string.trim().to_string())
            .ok()
    } else {
        None
    }
}

pub fn expand_package_path<'a>(filename: &'a str, base_dir: Option<&Path>) -> Cow<'a, str> {
    static RE: Lazy<Regex> = Lazy::new(|| Regex::new("^package://(\\w+)/").unwrap());

    if filename.starts_with("package://") {
        RE.replace(filename, |ma: &regex::Captures<'_>| {
            match rospack_find(&ma[1]) {
                Some(found_path) => found_path + "/",
                None => panic!("failed to find ros package {}", &ma[1]),
            }
        })
    } else if filename.starts_with("https://") || filename.starts_with("http://") {
        filename.into()
    } else if let Some(abs_path) = filename.strip_prefix("file://") {
        abs_path.into()
    } else if let Some(base_dir) = base_dir {
        let mut relative_path_from_urdf = base_dir.to_owned();
        relative_path_from_urdf.push(filename);
        relative_path_from_urdf
            .into_os_string()
            .into_string()
            .unwrap()
            .into()
    } else {
        filename.into()
    }
}

pub fn read_urdf_or_xacro_with_args<P>(input_path: P, args: &[(String, String)]) -> Result<Robot>
where
    P: AsRef<Path>,
{
    if let Some(ext) = input_path.as_ref().extension() {
        if ext == "xacro" {
            let urdf_utf = convert_xacro_to_urdf_with_args(input_path.as_ref(), args)?;
            read_from_string(&urdf_utf)
        } else {
            read_file(&input_path)
        }
    } else {
        Err(ErrorKind::Other("failed to get extension".to_owned()).into())
    }
}

pub fn read_urdf_or_xacro<P>(input_path: P) -> Result<Robot>
where
    P: AsRef<Path>,
{
    read_urdf_or_xacro_with_args(input_path, &[])
}

#[test]
fn it_works() {
    // test only for not packages
    assert_eq!(expand_package_path("home/aaa", None), "home/aaa");
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new(""))),
        "home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new("/var"))),
        "/var/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("/home/aaa.obj", Some(Path::new(""))),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("file:///home/aaa.obj", Some(Path::new("/var"))),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("http://aaa.obj", Some(Path::new("/var"))),
        "http://aaa.obj"
    );
    assert_eq!(
        expand_package_path("https://aaa.obj", Some(Path::new("/var"))),
        "https://aaa.obj"
    );
    assert!(read_urdf_or_xacro("sample.urdf").is_ok());
    assert!(read_urdf_or_xacro("sample_urdf").is_err());
}
