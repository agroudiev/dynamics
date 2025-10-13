#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3D(pub(crate) nalgebra::Vector3<f64>);

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(nalgebra::Vector3::new(x, y, z))
    }

    pub fn zeros() -> Self {
        Self(nalgebra::Vector3::zeros())
    }

    pub fn as_slice(&self) -> &[f64; 3] {
        self.0.as_slice().try_into().unwrap()
    }
}
