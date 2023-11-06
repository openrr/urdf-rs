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
    let filename = filename.as_ref();
    let output = Command::new("rosrun")
        .args(["xacro", "xacro", "--inorder"])
        .arg(filename)
        .args(args.iter().map(|(k, v)| format!("{}:={}", k, v)))
        .output()
        .or_else(|_| {
            Command::new("xacro")
                .arg(filename)
                .args(args.iter().map(|(k, v)| format!("{}:={}", k, v)))
                .output()
        })
        .map_err(|e| {
            format!("failed to execute xacro; consider installing xacro by `apt-get install ros-*-xacro`: {e}")
        })?;
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?)
    } else {
        Err(ErrorKind::Command {
            msg: format!("failed to xacro for {}", filename.display()),
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

pub fn rospack_find(package: &str) -> Result<String> {
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
        .map_err(|e| {
            format!("failed to execute neither `rospack` nor `ros2 pkg`; consider installing ROS or replacing 'package://' with path: {e}")
        })?;
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?.trim().to_string())
    } else {
        Err(ErrorKind::Command {
            msg: format!("failed to find ros package {package}"),
            stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        }
        .into())
    }
}

// Note: We return Result here, although there is currently no branch that returns an error.
// This is to avoid a breaking change when changing the error about package:// from panic to error.
pub fn expand_package_path<'a>(filename: &'a str, base_dir: Option<&Path>) -> Result<Cow<'a, str>> {
    static RE: Lazy<Regex> = Lazy::new(|| Regex::new("^package://(\\w+)/").unwrap());

    Ok(if filename.starts_with("package://") {
        RE.replace(filename, |ma: &regex::Captures<'_>| {
            // TODO: It's better to propagate the error to the caller,
            // but regex doesn't provide API like try_replace.
            let found_path = rospack_find(&ma[1]).unwrap();
            found_path + "/"
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
    })
}

pub fn read_urdf_or_xacro_with_args<P>(input_path: P, args: &[(String, String)]) -> Result<Robot>
where
    P: AsRef<Path>,
{
    let input_path = input_path.as_ref();
    if let Some(ext) = input_path.extension() {
        if ext == "xacro" {
            let urdf_utf = convert_xacro_to_urdf_with_args(input_path, args)?;
            read_from_string(&urdf_utf)
        } else {
            read_file(input_path)
        }
    } else {
        Err(ErrorKind::Other(format!(
            "failed to get extension from {}",
            input_path.display()
        ))
        .into())
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
    assert_eq!(expand_package_path("home/aaa", None).unwrap(), "home/aaa");
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new(""))).unwrap(),
        "home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new("/var"))).unwrap(),
        "/var/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("/home/aaa.obj", Some(Path::new(""))).unwrap(),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("file:///home/aaa.obj", Some(Path::new("/var"))).unwrap(),
        "/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("http://aaa.obj", Some(Path::new("/var"))).unwrap(),
        "http://aaa.obj"
    );
    assert_eq!(
        expand_package_path("https://aaa.obj", Some(Path::new("/var"))).unwrap(),
        "https://aaa.obj"
    );
    assert!(read_urdf_or_xacro("sample.urdf").is_ok());
    assert!(read_urdf_or_xacro("sample_urdf").is_err());
}
