use deserialize::Robot;
use errors::*;
use funcs::*;

use std::path::Path;
use std::process::Command;
use regex;
use regex::Regex;

pub fn convert_xacro_to_urdf<P>(filename: P) -> Result<String>
where
    P: AsRef<Path>,
{
    let output = Command::new("rosrun")
        .args(
            &[
                "xacro",
                "xacro",
                "--inorder",
                filename.as_ref().to_str().ok_or(
                    "failed to get str fro filename",
                )?,
            ],
        )
        .output()
        .expect(
            "failed to execute xacro. install by apt-get install ros-*-xacro",
        );
    if output.status.success() {
        Ok(String::from_utf8(output.stdout)?)
    } else {
        Err(UrdfError::Command("faild to xacro".to_owned()))
    }
}


pub fn rospack_find(package: &str) -> Option<String> {
    let output = Command::new("rospack")
        .arg("find")
        .arg(package)
        .output()
        .expect("rospack find failed");
    if output.status.success() {
        String::from_utf8(output.stdout)
            .map(|string| string.trim().to_string())
            .ok()
    } else {
        None
    }
}


pub fn expand_package_path(filename: &str, base_dir: Option<&Path>) -> String {
    if filename.starts_with("package://") {
        let re = Regex::new("^package://(\\w+)/").unwrap();
        re.replace(
            filename,
            |ma: &regex::Captures| match rospack_find(&ma[1]) {
                Some(found_path) => found_path + "/",
                None => panic!("failed to find ros package {}", &ma[1]),
            },
        ).to_string()
    } else {
        let mut relative_path_from_urdf = base_dir.unwrap_or(Path::new("")).to_owned();
        relative_path_from_urdf.push(filename);
        relative_path_from_urdf.to_str().unwrap().to_owned()
    }
}



pub fn read_urdf_or_xacro<P>(input_path: P) -> Result<Robot>
where
    P: AsRef<Path>,
{
    if let Some(ext) = input_path.as_ref().extension() {
        if ext == "xacro" {
            let urdf_utf = try!(convert_xacro_to_urdf(input_path.as_ref()));
            read_from_string(&urdf_utf)
        } else {
            read_file(&input_path)
        }
    } else {
        return Err(UrdfError::Command("failed to get extension".to_owned()));
    }
}

#[test]
fn it_works() {
    // test only for not packages
    assert_eq!(expand_package_path("home/aaa", None), "home/aaa");
    assert_eq!(
        expand_package_path("home/aaa.obj", Some(Path::new("/var"))),
        "/var/home/aaa.obj"
    );
    assert_eq!(
        expand_package_path("/home/aaa.obj", Some(Path::new(""))),
        "/home/aaa.obj"
    );
    assert!(read_urdf_or_xacro("sample.urdf").is_ok());
    assert!(read_urdf_or_xacro("sample_urdf").is_err());
}
