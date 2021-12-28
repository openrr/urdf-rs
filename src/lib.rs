#![doc = include_str!("../README.md")]
#![warn(missing_debug_implementations, rust_2018_idioms)]

mod errors;
pub use errors::*;

mod deserialize;
pub use deserialize::*;

mod funcs;
pub use funcs::*;

pub mod utils;
