//! Defines **joint limits** and related operations.

use dynamics_spatial::configuration::Configuration;

/// A joint limit, defining the physical constraints and dynamical properties of a joint.
#[derive(Clone, Debug, PartialEq)]
pub struct JointLimits {
    /// Maximum torque/force that can be applied by the joint.
    pub effort: Configuration,
    /// Maximum velocity of the joint.
    pub velocity: Configuration,
    /// Minimum configuration of the joint.
    pub min_configuration: Configuration,
    /// Maximum configuration of the joint.
    pub max_configuration: Configuration,
    /// Joint friction coefficient.
    pub friction: Configuration,
    /// Joint damping coefficient.
    pub damping: Configuration,
    /// Dry friction loss.
    pub friction_loss: f64,
}

impl JointLimits {
    /// Creates a new [`JointLimits`] with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `effort` - Maximum torque/force that can be applied by the joint.
    /// * `velocity` - Maximum velocity of the joint.
    /// * `min_configuration` - Minimum configuration of the joint.
    /// * `max_configuration` - Maximum configuration of the joint.
    /// * `friction` - Joint friction coefficient.
    /// * `damping` - Joint damping coefficient.
    /// * `friction_loss` - Dry friction loss.
    ///
    /// # Returns
    /// A new [`JointLimits`] object.
    #[must_use]
    pub fn new(
        effort: Configuration,
        velocity: Configuration,
        min_configuration: Configuration,
        max_configuration: Configuration,
        friction: Configuration,
        damping: Configuration,
        friction_loss: f64,
    ) -> Self {
        Self {
            effort,
            velocity,
            min_configuration,
            max_configuration,
            friction,
            damping,
            friction_loss,
        }
    }

    /// Creates a new unbounded [`JointLimits`], with infinite limits and zero friction/damping.
    ///
    /// # Returns
    /// A new unbounded [`JointLimits`] object.
    pub fn new_unbounded(nq: usize) -> Self {
        Self {
            effort: Configuration::from_element(nq, f64::INFINITY),
            velocity: Configuration::from_element(nq, f64::INFINITY),
            min_configuration: Configuration::from_element(nq, f64::NEG_INFINITY),
            max_configuration: Configuration::from_element(nq, f64::INFINITY),
            friction: Configuration::zeros(nq),
            damping: Configuration::zeros(nq),
            friction_loss: 0.0,
        }
    }
}
