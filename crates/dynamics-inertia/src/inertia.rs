//! Structures to represent the inertia of a rigid body.

use dynamics_spatial::{
    force::SpatialForce,
    motion::SpatialMotion,
    se3::{ActSE3, SE3},
    symmetric3::Symmetric3,
    vector3d::Vector3D,
};
use std::ops::{Add, AddAssign, Mul};

/// A data structure that contains the information about the inertia of a rigid body (mass, center of mass, and inertia matrix).
#[derive(Clone, Debug, Default)]
pub struct Inertia {
    /// The mass of the object.
    pub mass: f64,
    /// The center of mass of the object.
    pub com: Vector3D,
    /// Rotational inertia matrix at the center of mass.
    pub inertia: Symmetric3,
}

impl Inertia {
    /// Creates a new [`Inertia`] object with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the object.
    /// * `com` - The center of mass of the object.
    /// * [`Inertia`] - The rotational inertia matrix of the object at the center of mass.
    #[must_use]
    pub fn new(mass: f64, com: Vector3D, inertia: Symmetric3) -> Self {
        Self { mass, com, inertia }
    }

    /// Creates a new [`Inertia`] object with zero mass, zero center of mass, and zero inertia matrix.
    ///
    /// # Returns
    /// A new [`Inertia`] object with all properties set to zero.
    #[must_use]
    pub fn zeros() -> Self {
        Self {
            mass: 0.0,
            com: Vector3D::zeros(),
            inertia: Symmetric3::zeros(),
        }
    }

    /// Creates a new [`Inertia`] object representing an ellipsoid with the given mass and semi-axis lengths.
    ///
    /// # Arguments
    /// * `mass` - The mass of the ellipsoid.
    /// * `x` - The semi-axis length along the x-axis.
    /// * `y` - The semi-axis length along the y-axis.
    /// * `z` - The semi-axis length along the z-axis.
    ///
    /// # Returns
    /// A new [`Inertia`] object representing an ellipsoid.
    pub fn from_ellipsoid(mass: f64, x: f64, y: f64, z: f64) -> Result<Self, InertiaError> {
        if mass <= 0.0 {
            return Err(InertiaError::InvalidParameter("mass".to_string()));
        }
        if x <= 0.0 {
            return Err(InertiaError::InvalidParameter("x".to_string()));
        }
        if y <= 0.0 {
            return Err(InertiaError::InvalidParameter("y".to_string()));
        }
        if z <= 0.0 {
            return Err(InertiaError::InvalidParameter("z".to_string()));
        }

        let a = mass * (y.powi(2) + z.powi(2)) / 5.0;
        let b = mass * (x.powi(2) + z.powi(2)) / 5.0;
        let c = mass * (x.powi(2) + y.powi(2)) / 5.0;
        let inertia_matrix = Symmetric3::new(a, 0.0, b, 0.0, 0.0, c);
        Ok(Self::new(mass, Vector3D::zeros(), inertia_matrix))
    }

    /// Creates a new [`Inertia`] object representing a sphere with the given mass and radius.
    ///
    /// # Arguments
    ///
    /// * `mass` - The mass of the sphere.
    /// * `radius` - The radius of the sphere.
    ///
    /// # Returns
    /// A new [`Inertia`] object representing a sphere.
    pub fn from_sphere(mass: f64, radius: f64) -> Result<Self, InertiaError> {
        Inertia::from_ellipsoid(mass, radius, radius, radius)
    }
}

impl Mul<&SpatialMotion> for &Inertia {
    type Output = SpatialForce;

    fn mul(self, rhs: &SpatialMotion) -> Self::Output {
        let linear = self.mass * (rhs.translation() - self.com.cross(&rhs.rotation()));
        let angular = &self.inertia * &rhs.rotation() + self.com.cross(&linear);
        SpatialForce::from_parts(linear, angular)
    }
}

impl Add for Inertia {
    type Output = Inertia;

    fn add(self, rhs: Inertia) -> Self::Output {
        let total_mass = self.mass + rhs.mass;
        let total_mass_inv = 1.0 / total_mass.max(f64::EPSILON);
        let ab = self.com - rhs.com;
        Inertia::new(
            total_mass,
            (self.mass * self.com + rhs.mass * rhs.com) * total_mass_inv,
            self.inertia + rhs.inertia
                - (self.mass * rhs.mass * total_mass_inv) * Symmetric3::skew_square(ab),
        )
    }
}

impl AddAssign for Inertia {
    fn add_assign(&mut self, rhs: Inertia) {
        *self = self.clone() + rhs;
    }
}

impl ActSE3 for Inertia {
    fn act(&self, t: &SE3) -> Self {
        Inertia::new(
            self.mass,
            t.translation() + t.rotation() * &self.com,
            self.inertia.rotate(&t.rotation()),
        )
    }

    fn act_inv(&self, se3: &SE3) -> Self {
        let rotation_inv = se3.rotation().transpose();
        Inertia::new(
            self.mass,
            rotation_inv * &(self.com - se3.translation()),
            self.inertia.rotate(&rotation_inv),
        )
    }
}

/// An error type for the [`Inertia`] struct.
pub enum InertiaError {
    InvalidParameter(String),
}

impl std::fmt::Display for InertiaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InertiaError::InvalidParameter(param) => {
                write!(f, "Invalid parameter: '{param}' must be positive.")
            }
        }
    }
}

impl std::fmt::Debug for InertiaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "InertiaError: {self}")
    }
}
