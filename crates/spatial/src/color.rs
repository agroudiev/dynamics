use nalgebra::Vector4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color(pub(crate) Vector4<f64>);

impl Color {
    pub fn new(r: f64, g: f64, b: f64, a: f64) -> Self {
        Self(Vector4::new(r, g, b, a))
    }

    pub fn black() -> Self {
        Self(Vector4::new(0.0, 0.0, 0.0, 1.0))
    }

    pub fn white() -> Self {
        Self(Vector4::new(1.0, 1.0, 1.0, 1.0))
    }

    pub fn transparent() -> Self {
        Self(Vector4::new(0.0, 0.0, 0.0, 0.0))
    }

    pub fn as_slice(&self) -> &[f64; 4] {
        self.0.as_slice().try_into().unwrap()
    }
}
