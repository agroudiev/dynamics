use std::vec;

use crate::data::{Data, PyData};
use crate::errors::AlgorithmError;
use crate::model::{Model, PyModel};
use dynamics_spatial::configuration::Configuration;
use dynamics_spatial::se3::SE3;
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Computes the forward kinematics of the robot model.
///
/// It updates the joint data and placements in the world frame.
///
/// # Arguments
///
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
/// * `q` - The configuration of the robot of size `nq`.
/// * `v` - The velocity configuration of the robot of size `nv` (optional).
/// * `a` - The acceleration configuration of the robot of size `nv` (optional).
///
/// # Returns
///
/// * Updates `data.joint_data` and `data.joint_placements` in place if successful.
/// * Returns an [`AlgorithmError`] if there was an error.
pub fn forward_kinematics(
    model: &Model,
    data: &mut Data,
    q: &Configuration,
    v: &Option<Configuration>,
    a: &Option<Configuration>,
) -> Result<(), AlgorithmError> {
    // check if configurations are of the right size
    q.check_size("q", model.nq)
        .map_err(AlgorithmError::ConfigurationError)?;
    v.as_ref()
        .map(|v| {
            v.check_size("v", model.nv)
                .map_err(AlgorithmError::ConfigurationError)
        })
        .transpose()?;
    a.as_ref()
        .map(|a| {
            a.check_size("a", model.nv)
                .map_err(AlgorithmError::ConfigurationError)
        })
        .transpose()?;

    // update the joints data
    let mut offset = 0;
    for id in 0..model.njoints() {
        let joint_data = &mut data.joint_data[id];
        let joint_model = &model.joint_models[id];
        let q_joint = q.rows(offset, joint_model.nq());
        match joint_data.update(joint_model, &q_joint) {
            Ok(()) => {}
            Err(e) => {
                return Err(AlgorithmError::JointError(model.joint_names[id].clone(), e));
            }
        }
        offset += joint_model.nq();
    }

    // update the placements of the joints in the world frame
    // by traversing the joint tree
    data.joint_placements = vec![SE3::identity()];

    for joint_id in 1..model.njoints() {
        // get the placement of the parent joint in the world frame
        let parent_id = model.joint_parents[joint_id]; // we checked that the parent existed before
        let parent_placement = data.joint_placements[parent_id];

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

/// Updates the placements of all frames in the world frame, based on the current joint placements.
///
/// This function must be called after `forward_kinematics`.
///
/// # Arguments
/// * `model` - The robot model.
/// * `data` - The data structure that contains the joint data.
///
/// # Returns
///
/// Updates `data.frame_placements` in place if successful.
pub fn update_frame_placements(model: &Model, data: &mut Data) {
    for frame_id in 1..model.nframes() {
        let frame = &model.frames[frame_id];
        let parent_joint_placement = data.joint_placements[frame.parent_joint];
        data.frame_placements[frame_id] = parent_joint_placement * frame.placement;
    }
}

#[pyfunction(name = "forward_kinematics", signature=(model, data, q, v=None, a=None))]
pub fn py_forward_kinematics(
    model: &PyModel,
    data: &mut PyData,
    q: PyReadonlyArray1<f64>,
    v: Option<PyReadonlyArray1<f64>>,
    a: Option<PyReadonlyArray1<f64>>,
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
        None => return Err(PyValueError::new_err("Failed to convert 'q' to slice")),
    };
    let q = Configuration::from_row_slice(q);

    let v = match v {
        Some(v_array) => {
            let v_array = v_array.as_array();
            if v_array.shape() != [model.inner.nv] {
                return Err(PyValueError::new_err(format!(
                    "Invalid input size. Expected a configuration of size {}, got {:?}",
                    model.inner.nv,
                    v_array.shape()
                )));
            }
            let v_slice = match v_array.as_slice() {
                Some(slice) => slice,
                None => return Err(PyValueError::new_err("Failed to convert 'v' to slice")),
            };
            Some(Configuration::from_row_slice(v_slice))
        }
        None => None,
    };

    let a = match a {
        Some(a_array) => {
            let a_array = a_array.as_array();
            if a_array.shape() != [model.inner.nv] {
                return Err(PyValueError::new_err(format!(
                    "Invalid input size. Expected a configuration of size {}, got {:?}",
                    model.inner.nv,
                    a_array.shape()
                )));
            }
            let a_slice = match a_array.as_slice() {
                Some(slice) => slice,
                None => return Err(PyValueError::new_err("Failed to convert 'a' to slice")),
            };
            Some(Configuration::from_row_slice(a_slice))
        }
        None => None,
    };

    forward_kinematics(&model.inner, &mut data.inner, &q, &v, &a)
        .map_err(|e| PyValueError::new_err(format!("Forward kinematics failed: {e:?}")))?;
    Ok(())
}

#[pyfunction(name = "update_frame_placements", signature=(model, data))]
pub fn py_update_frame_placements(model: &PyModel, data: &mut PyData) {
    update_frame_placements(&model.inner, &mut data.inner);
}
