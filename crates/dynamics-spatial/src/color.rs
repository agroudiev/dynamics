//! Defines RGBA color representation, used for geometric visualization.

use nalgebra::Vector4;

#[derive(Debug, Clone, Copy, PartialEq)]
/// RGBA color representation.
pub struct Color(pub(crate) Vector4<f64>);

impl Color {
    /// Creates a new `Color` from RGBA components.
    #[must_use]
    pub fn new(r: f64, g: f64, b: f64, a: f64) -> Self {
        Self(Vector4::new(r, g, b, a))
    }

    /// Black color with full opacity.
    #[must_use]
    pub fn black() -> Self {
        Self(Vector4::new(0.0, 0.0, 0.0, 1.0))
    }

    /// White color with full opacity.
    #[must_use]
    pub fn white() -> Self {
        Self(Vector4::new(1.0, 1.0, 1.0, 1.0))
    }

    /// Transparent "black" color.
    #[must_use]
    pub fn transparent() -> Self {
        Self(Vector4::new(0.0, 0.0, 0.0, 0.0))
    }

    #[must_use]
    /// Return the color vector as a slice.
    pub fn as_slice(&self) -> &[f64; 4] {
        self.0.as_slice().try_into().unwrap()
    }
}
