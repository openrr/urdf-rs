# urdf-rs

[URDF](http://wiki.ros.org/urdf) parser using [serde-xml-rs](https://github.com/RReverser/serde-xml-rs) for rust.

Only [link](http://wiki.ros.org/urdf/XML/link) and [joint](http://wiki.ros.org/urdf/XML/joint) are supported.

You can access urdf elements like below sample.

```rust
extern crate urdf_rs;
let urdf_robo = urdf_rs::read_file("sample.urdf").unwrap();
let links = urdf_robo.links;
println!("{}", links.visual.origin.xyz);
let joints = urdf_robo.joints;
```
