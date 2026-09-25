# urdf-rs

[![Build Status](https://img.shields.io/github/actions/workflow/status/openrr/urdf-rs/ci.yml?branch=main&logo=github)](https://github.com/openrr/urdf-rs/actions) [![codecov](https://codecov.io/gh/openrr/urdf-rs/branch/main/graph/badge.svg)](https://codecov.io/gh/openrr/urdf-rs) [![crates.io](https://img.shields.io/crates/v/urdf-rs.svg?logo=rust)](https://crates.io/crates/urdf-rs) [![docs](https://docs.rs/urdf-rs/badge.svg)](https://docs.rs/urdf-rs) [![discord](https://dcbadge.vercel.app/api/server/8DAFFKc88B?style=flat)](https://discord.gg/8DAFFKc88B)

[URDF](http://wiki.ros.org/urdf) parser for Rust.

Only [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) are supported.

## URDF versions

The `version` attribute of `<robot>` is supported, like [urdfdom](https://github.com/ros/urdfdom). If it is omitted, version 1.0 is assumed.

- 1.1: The `quat_xyzw` attribute of `<origin>` is supported. It is converted to `rpy` when parsing.
- 1.2: The `acceleration`, `deceleration` and `jerk` attributes of `<limit>` are supported. Omitted limits mean no limit and are represented as infinity. Negative limits, `upper` smaller than `lower`, and non-positive or non-finite geometry dimensions are errors.

## Example

You can access urdf elements like below example.

```rust
let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
let links = urdf_robot.links;
println!("{:?}", links[0].visual[0].origin.xyz);
let joints = urdf_robot.joints;
println!("{:?}", joints[0].origin.xyz);
```

## `OpenRR` Community

[Here](https://discord.gg/8DAFFKc88B) is a discord server for `OpenRR` users and developers.
