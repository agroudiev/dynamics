//! This crate is part of the `dynamics` ecosystem, and is not intended for direct use.
//!
//! This crate provides parsers for various robot description formats.
//! Currently, it supports URDF (Unified Robot Description Format) parsing.

pub mod errors;
pub mod new_urdf;
pub mod urdf;

#[cfg(test)]
mod tests;
