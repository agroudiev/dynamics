//! This module contains the implementation of the forward dynamics algorithms.
//! The main algorithm implemented here is the Articulated Body Algorithm (ABA).
//! The ABA computes the joint accelerations required to achieve a given motion of the robot
//! given its configuration, velocity, and torques.

use crate::model::Model;
use crate::{data::Data, errors::AlgorithmError};
use dynamics_spatial::configuration::Configuration;

/// Computes the forward dynamics of the robot model using the Articulated Body Algorithm (ABA).
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
/// * `v` - The velocity of the robot.
/// * `tau` - The joint torques.
///
/// # Returns
///
/// * `Ok(ddq)` if the forward dynamics was successful.
/// * `Err(ConfigurationError)` if there was an error.
pub fn forward_dynamics(
    model: &Model,
    _data: &mut Data,
    q: &Configuration,
    v: &Configuration,
    tau: &Configuration,
) -> Result<(), AlgorithmError> {
    q.check_size("q", model.nq)
        .map_err(AlgorithmError::ConfigurationError)?;
    v.check_size("v", model.nv)
        .map_err(AlgorithmError::ConfigurationError)?;
    tau.check_size("tau", model.nv)
        .map_err(AlgorithmError::ConfigurationError)?;

    unimplemented!()
}
