use nalgebra::{Matrix6, Vector6};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector6D(pub(crate) Vector6<f64>);

impl Vector6D {
    pub fn new(x: f64, y: f64, z: f64, w: f64, v: f64, u: f64) -> Self {
        Self(Vector6::new(x, y, z, w, v, u))
    }

    pub fn zeros() -> Self {
        Self(Vector6::zeros())
    }

    pub fn as_slice(&self) -> &[f64; 6] {
        self.0.as_slice().try_into().unwrap()
    }

    pub fn as_diagonal(&self) -> Matrix6<f64> {
        Matrix6::from_diagonal(&self.0)
    }
}
