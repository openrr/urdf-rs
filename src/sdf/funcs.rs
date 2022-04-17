use crate::errors::*;
use crate::sdf::deserialize::*;

use std::path::Path;

/// Read SDFormat file and create Sdf instance
///
/// # Examples
///
/// ```
/// let sdf = urdf_rs::sdf::read_file("samples/minimal.sdf").unwrap();
/// println!("{:?}", sdf);
/// ```
pub fn read_file<P: AsRef<Path>>(path: P) -> Result<Sdf> {
    read_from_string(&std::fs::read_to_string(path)?)
}

/// Read from string instead of file.
///
///
/// # Examples
///
/// ```
/// let s = r##"
///    <?xml version='1.0'?>
///    <sdf version='1.9'>
///    <model name='my_model'>
///      <link name='link'/>
///    </model>
///    </sdf>
/// "##;
/// let sdf = urdf_rs::sdf::read_from_string(s).unwrap();
/// let model = urdf_rs::sdf::Model { name: "my_model".to_string() };
/// assert_eq!(sdf.models(), vec![&model]);
/// ```
pub fn read_from_string(string: &str) -> Result<Sdf> {
    // let sdf_elem: xml::Element = string.parse().map_err(UrdfError::new)?;
    // if (sdf_elem.name != "sdf") {
    //     return UrdfError
    // }
    let mut sdf: SDFormat = serde_xml_rs::from_str(&string).map_err(UrdfError::new)?;
    if sdf.worlds.len() > 0 {
        return Ok(Sdf::Worlds(sdf.worlds));
    }
    let model = sdf.models.pop();
    match model {
        Some(m) => Ok(Sdf::Model(m)),
        None => panic!("TODO: Custom error types."),
    }
}

#[test]
fn models_parsed() {
    let s = r##"
<?xml version='1.0'?>
<sdf version='1.9'>
<model name='my_model'>
  <link name='link'/>
</model>
</sdf>
"##;
    let sdf = read_from_string(s).unwrap();

    if let Sdf::Model(root) = sdf {
        assert_eq!(root.name, "my_model");
    } else {
        unreachable!()
    }

    let s = r##"
<?xml version='1.0'?>
<sdf version='1.9'>
<world name='earth'>
  <model name='my_model'>
    <link name='link'/>
  </model>
</world>
<world name='mars'>
  <model name='discovery'>
    <link name='link'/>
  </model>
</world>
</sdf>
"##;

    let sdf = read_from_string(s).unwrap();
    let model_names: Vec<&String> = sdf.models().iter().map(|m| &m.name).collect();
    assert_eq!(model_names, vec!["my_model", "discovery"]);
}
