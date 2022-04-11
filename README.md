# urdf-rs

[![Build Status](https://img.shields.io/github/workflow/status/openrr/urdf-rs/CI/main)](https://github.com/openrr/urdf-rs/actions) [![crates.io](https://img.shields.io/crates/v/urdf-rs.svg)](https://crates.io/crates/urdf-rs) [![docs](https://docs.rs/urdf-rs/badge.svg)](https://docs.rs/urdf-rs)

[URDF](http://wiki.ros.org/urdf) parser using [serde-xml-rs](https://github.com/RReverser/serde-xml-rs) for rust.

Only [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) are supported.

## Example

You can access urdf elements like below example.

```rust
let urdf_robo = urdf_rs::urdf::read_file("samples/sample.urdf").unwrap();
let links = urdf_robo.links;
println!("{:?}", links[0].visual[0].origin.xyz);
let joints = urdf_robo.joints;
println!("{:?}", joints[0].origin.xyz);
```

## Contributors

<a href="https://github.com/openrr/urdf-rs/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=openrr/urdf-rs" />
</a>

Made with [contrib.rocks](https://contrib.rocks).
