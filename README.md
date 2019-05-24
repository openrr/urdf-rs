# urdf-rs [![Build Status](https://travis-ci.org/OTL/urdf-rs.svg?branch=master)](https://travis-ci.org/OTL/urdf-rs)  [![crates.io](https://img.shields.io/crates/v/urdf-rs.svg)](https://crates.io/crates/urdf-rs)


[URDF](http://wiki.ros.org/urdf) parser using [serde-xml-rs](https://github.com/RReverser/serde-xml-rs) for rust.

Only [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) are supported.

[Documentation](https://docs.rs/urdf-rs/)

## Example

You can access urdf elements like below example.

```rust
extern crate urdf_rs;
let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
let links = urdf_robo.links;
println!("{:?}", links[0].visual.origin.xyz);
let joints = urdf_robo.joints;
println!("{:?}", joints[0].origin.xyz);
```

## Contributors

* Johan Andersson
* Tom Olsson
