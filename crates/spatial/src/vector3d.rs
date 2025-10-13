use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, PartialEq)]
/// A 3D vector, commonly used for positions.
pub struct Vector3D(pub(crate) Vector3<f64>);

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z))
    }

    pub fn zeros() -> Self {
        Self(Vector3::zeros())
    }

    pub fn as_slice(&self) -> &[f64; 3] {
        self.0.as_slice().try_into().unwrap()
    }

    pub fn norm(&self) -> f64 {
        self.0.norm()
    }
}

impl std::ops::Add for Vector3D {
    type Output = Vector3D;

    fn add(self, rhs: Self) -> Self::Output {
        Vector3D(self.0 + rhs.0)
    }
}

impl std::ops::Sub for Vector3D {
    type Output = Vector3D;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3D(self.0 - rhs.0)
    }
}

impl std::ops::Mul for Vector3D {
    type Output = Vector3D;

    fn mul(self, rhs: Self) -> Self::Output {
        Vector3D(self.0.component_mul(&rhs.0))
    }
}

impl std::ops::Mul<f64> for Vector3D {
    type Output = Vector3D;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector3D(self.0 * rhs)
    }
}
