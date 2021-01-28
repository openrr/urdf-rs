//! # urdf-rs
//!
//! [![Build Status](https://img.shields.io/github/workflow/status/openrr/urdf-rs/CI/main)](https://github.com/openrr/urdf-rs/actions?query=workflow%3ACI+branch%3Amain)
//!
//! [URDF](http://wiki.ros.org/urdf) parser using
//! [serde-xml-rs](https://github.com/RReverser/serde-xml-rs) for rust.
//!
//! Only [link](http://wiki.ros.org/urdf/XML/link) and
//! [joint](http://wiki.ros.org/urdf/XML/joint) are supported.
//!
//! # Examples
//!
//! You can access urdf elements like below example.
//!
//! ```
//! let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
//! let links = urdf_robo.links;
//! println!("{:?}", links[0].visual[0].origin.xyz);
//! let joints = urdf_robo.joints;
//! println!("{:?}", joints[0].origin.xyz);
//! ```

mod errors;
pub use errors::*;

mod deserialize;
pub use deserialize::*;

mod funcs;
pub use funcs::*;

pub mod utils;
