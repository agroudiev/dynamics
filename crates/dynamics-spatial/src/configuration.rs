//! Defines **configurations** of multi-body systems and related operations.

use approx::{AbsDiffEq, RelativeEq};
use nalgebra::DVector;

use rand::{Rng, rngs::ThreadRng};
use std::{
    f64::consts::PI,
    fmt::Display,
    ops::{Add, Index, IndexMut, Mul},
};

#[cfg(feature = "python")]
use numpy::PyReadonlyArrayDyn;
#[cfg(feature = "python")]
use pyo3::prelude::*;

#[derive(Clone, Debug, PartialEq)]
/// Configuration of a multi-body system, represented as a vector of joint positions.
pub struct Configuration(pub(crate) DVector<f64>);

impl Configuration {
    /// Creates a new [`Configuration`] with the given size, initialized to zeros.
    /// # Arguments
    /// * `size` - The size of the configuration vector.
    /// # Returns
    /// A new [`Configuration`] object with all values set to zero.
    #[must_use]
    pub fn zeros(size: usize) -> Self {
        Configuration(DVector::zeros(size))
    }

    /// Creates a new [`Configuration`] with the given size, initialized to ones.
    /// # Arguments
    /// * `size` - The size of the configuration vector.
    /// # Returns
    /// A new [`Configuration`] object with all values set to one.
    #[must_use]
    pub fn ones(size: usize) -> Self {
        Configuration(DVector::from_element(size, 1.0))
    }

    /// Creates a new [`Configuration`] with the given size, initialized to the specified value.
    /// # Arguments
    /// * `size` - The size of the configuration vector.
    /// * `value` - The value to initialize each element of the configuration vector.
    /// # Returns
    /// A new [`Configuration`] object with all values set to the specified value.
    #[must_use]
    pub fn from_element(size: usize, value: f64) -> Self {
        Configuration(DVector::from_element(size, value))
    }

    /// Returns the length of the configuration vector.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Checks if the configuration vector is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns a slice of the configuration vector from `start` to `start + nrows - 1` (inclusive).
    ///
    /// The returned slice goes from index `start` to `start + nrows - 1`,
    /// included, and contains `nrows` elements.
    ///
    /// # Arguments
    /// * `start` - The starting index of the slice.
    /// * `nrows` - The number of rows to include in the slice.
    ///
    /// # Returns
    /// A new [`Configuration`] object containing the specified slice.
    #[must_use]
    pub fn rows(&self, start: usize, nrows: usize) -> Configuration {
        Configuration(self.0.rows(start, nrows).into_owned())
    }

    /// Updates a slice of the configuration vector starting from `start` with the values from another configuration.
    ///
    /// The slice to be updated starts at index `start` and has the same length as the provided `values` configuration.
    ///
    /// # Arguments
    /// * `start` - The starting index of the slice to be updated.
    /// * `values` - The configuration containing the new values to be copied.
    ///
    /// # Returns
    /// * `Ok(())` if the update was successful.
    /// * `Err(ConfigurationError)` if the sizes of the slices do not match.
    pub fn update_rows(
        &mut self,
        start: usize,
        values: &Configuration,
    ) -> Result<(), ConfigurationError> {
        if self.0.rows(start, values.len()).len() != values.0.len() {
            Err(ConfigurationError::MismatchedUpdateSize(
                self.0.rows(start, values.len()).len(),
                values.0.len(),
            ))
        } else {
            self.0.rows_mut(start, values.len()).copy_from(&values.0);
            Ok(())
        }
    }

    /// Creates a new [`Configuration`] from a slice of scalar values.
    /// # Arguments
    /// * `data` - A slice of scalar values.
    /// # Returns
    /// A new [`Configuration`] object containing the values from the slice.
    #[must_use]
    pub fn from_row_slice(data: &[f64]) -> Self {
        Configuration(DVector::from_row_slice(data))
    }

    /// Creates a new [`Configuration`] from a `NumPy` array.
    /// # Arguments
    /// * `array` - A read-only `NumPy` array of scalar values.
    /// # Returns
    /// A new [`Configuration`] object containing the values from the `NumPy` array.
    #[cfg(feature = "python")]
    pub fn from_pyarray(array: &PyReadonlyArrayDyn<f64>) -> Result<Configuration, PyErr> {
        let array = array.as_array();
        let flat: Vec<f64> = array.iter().copied().collect();
        Ok(Configuration::from_row_slice(&flat))
    }

    /// Concatenates multiple [`Configuration`] objects into a single configuration.
    /// # Arguments
    /// * `configs` - A slice of [`Configuration`] objects to concatenate.
    /// # Returns
    /// A new [`Configuration`] object containing all values from the input configurations.
    #[must_use]
    pub fn concat(configs: &[Configuration]) -> Configuration {
        let mut all_values = Vec::new();
        for config in configs {
            all_values.extend_from_slice(config.0.as_slice());
        }
        Configuration::from_row_slice(&all_values)
    }

    /// Generates a random [`Configuration`] within specified minimum and maximum bounds.
    ///
    /// # Arguments
    /// * `nq` - The size of the configuration vector.
    /// * `rng` - A mutable reference to a random number generator.
    /// * `min` - The minimum bounds for each element of the configuration.
    /// * `max` - The maximum bounds for each element of the configuration.
    /// # Returns
    /// A new [`Configuration`] object with random values within the specified bounds.
    pub fn random(
        nq: usize,
        rng: &mut ThreadRng,
        min: &Configuration,
        max: &Configuration,
    ) -> Self {
        let mut values = Vec::with_capacity(nq);
        for i in 0..nq {
            // if min is infinite, replace it with -2*pi
            let min_i = if min[i].is_infinite() && min[i] < 0.0 {
                -2.0 * PI
            } else {
                min[i]
            };
            // if max is infinite, replace it with 2*pi
            let max_i = if max[i].is_infinite() && max[i] > 0.0 {
                2.0 * PI
            } else {
                max[i]
            };
            values.push(rng.random_range(min_i..=max_i));
        }
        Configuration::from_row_slice(&values)
    }

    /// Checks if the size of the configuration matches the expected size.
    pub fn check_size(&self, name: &str, expected_size: usize) -> Result<(), ConfigurationError> {
        if self.len() != expected_size {
            Err(ConfigurationError::InvalidParameterSize(
                name.to_string(),
                expected_size,
                self.len(),
            ))
        } else {
            Ok(())
        }
    }
}

impl Index<usize> for Configuration {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl IndexMut<usize> for Configuration {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl AbsDiffEq for Configuration {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        f64::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.0.abs_diff_eq(&other.0, epsilon)
    }
}

impl RelativeEq for Configuration {
    fn default_max_relative() -> f64 {
        f64::default_max_relative()
    }

    fn relative_eq(&self, other: &Self, epsilon: f64, max_relative: f64) -> bool {
        self.0.relative_eq(&other.0, epsilon, max_relative)
    }
}

impl Add for Configuration {
    type Output = Configuration;

    fn add(self, rhs: Self) -> Self::Output {
        Configuration(self.0 + rhs.0)
    }
}

impl Add for &Configuration {
    type Output = Configuration;

    fn add(self, rhs: Self) -> Self::Output {
        Configuration(&self.0 + &rhs.0)
    }
}

impl Mul<f64> for &Configuration {
    type Output = DVector<f64>;

    fn mul(self, rhs: f64) -> Self::Output {
        &self.0 * rhs
    }
}

impl Display for Configuration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Configuration(")?;
        for i in 0..self.len() {
            write!(f, "{:.5}", self[i])?;
            if i < self.len() - 1 {
                write!(f, ", ")?;
            }
        }
        write!(f, ")")?;
        Ok(())
    }
}

/// Errors that can occur when working with configurations.
pub enum ConfigurationError {
    /// Error indicating that a parameter has an invalid size.
    /// * `name` - The name of the parameter.
    /// * `expected` - The expected size of the parameter.
    /// * `actual` - The actual size of the parameter.
    InvalidParameterSize(String, usize, usize),
    /// Error when updating configuration rows with mismatched sizes.
    /// * `expected` - The expected size of the parameter.
    /// * `actual` - The actual size of the parameter.
    MismatchedUpdateSize(usize, usize),
}

impl std::fmt::Display for ConfigurationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ConfigurationError::InvalidParameterSize(name, expected, actual) => {
                write!(
                    f,
                    "Parameter '{name}' expected configuration size {expected}, but got {actual}"
                )
            }
            ConfigurationError::MismatchedUpdateSize(expected, actual) => {
                write!(
                    f,
                    "Mismatched sizes when updating configuration rows. Expected size {expected}, got {actual}."
                )
            }
        }
    }
}

impl std::fmt::Debug for ConfigurationError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{self}")
    }
}

impl std::error::Error for ConfigurationError {}
