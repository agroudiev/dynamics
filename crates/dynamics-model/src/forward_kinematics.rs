use std::vec;

use crate::data::{Data, PyData};
use crate::model::{Model, PyModel};
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use spatial::configuration::{Configuration, ConfigurationError};
use spatial::se3::SE3;

/// Computes the forward kinematics of the robot model.
///
/// It updates the joint data and placements in the world frame.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot.
///
/// # Returns
///
/// * `Ok(())` if the forward kinematics was successful.
/// * `Err(ConfigurationError)` if there was an error.
pub fn forward_kinematics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
) -> Result<(), ConfigurationError> {
    // check if q is of the right size
    if q.len() != model.nq {
        return Err(ConfigurationError::InvalidSize(
            "q".to_string(),
            model.nq,
            q.len(),
        ));
    }

    // update the joints data
    let mut offset = 0;
    for id in 0..model.joint_models.len() {
        let joint_data = &mut data.joint_data[id];
        let joint_model = &model.joint_models[id];
        let q_joint = q.rows(offset, joint_model.nq());
        match joint_data.update(joint_model, &q_joint) {
            Ok(_) => {}
            Err(e) => unimplemented!("handle joint data update error: {:?}", e),
        }
        offset += joint_model.nq();
    }

    // update the placements of the joints in the world frame
    // by traversing the joint tree
    data.joint_placements = vec![SE3::identity()];

    for joint_id in 0..model.joint_models.len() {
        let parent_id = model.joint_parents.get(joint_id).unwrap(); // we checked that the parent existed before
        // get the placement of the parent join in the world frame
        let parent_placement = data.joint_placements[*parent_id];
        // get the placement of the joint in the parent frame
        let local_joint_placement = model.joint_placements[joint_id];
        // get the joint transformation
        let joint_data = &data.joint_data[joint_id];
        let joint_placement = joint_data.get_joint_placement();
        // compute the placement of the joint in the world frame
        data.joint_placements.insert(
            joint_id,
            parent_placement * local_joint_placement * joint_placement,
        );
    }

    Ok(())
}

#[pyfunction(name = "forward_kinematics")]
pub fn py_forward_kinematics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArray1<f64>,
) -> PyResult<()> {
    let q = q.as_array();
    if q.shape() != [model.inner.nq] {
        return Err(PyValueError::new_err(format!(
            "Invalid input size. Expected a configuration of size {}, got {:?}",
            model.inner.nq,
            q.shape()
        )));
    }
    let q = match q.as_slice() {
        Some(slice) => slice,
        None => return Err(PyValueError::new_err("Failed to convert q to slice")),
    };
    let q = Configuration::from_row_slice(q);

    forward_kinematics(&model.inner, &mut data.inner, &q)
        .map_err(|e| PyValueError::new_err(format!("Forward kinematics failed: {:?}", e)))?;
    Ok(())
}
