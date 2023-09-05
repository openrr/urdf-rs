# urdf-rs

[![Build Status](https://img.shields.io/github/actions/workflow/status/openrr/urdf-rs/ci.yml?branch=main&logo=github)](https://github.com/openrr/urdf-rs/actions) [![crates.io](https://img.shields.io/crates/v/urdf-rs.svg?logo=rust)](https://crates.io/crates/urdf-rs) [![docs](https://docs.rs/urdf-rs/badge.svg)](https://docs.rs/urdf-rs) [![discord](https://dcbadge.vercel.app/api/server/8DAFFKc88B?style=flat)](https://discord.gg/8DAFFKc88B)

[URDF](http://wiki.ros.org/urdf) parser for Rust.

Only [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) are supported.

## Example

You can access urdf elements like below example.

```rust
let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
let links = urdf_robot.links;
println!("{:?}", links[0].visual[0].origin.xyz);
let joints = urdf_robot.joints;
println!("{:?}", joints[0].origin.xyz);
```

## Contributors

<a href="https://github.com/openrr/urdf-rs/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=openrr/urdf-rs" />
</a>

Made with [contrib.rocks](https://contrib.rocks).

## `OpenRR` Community

[Here](https://discord.gg/8DAFFKc88B) is a discord server for `OpenRR` users and developers.
