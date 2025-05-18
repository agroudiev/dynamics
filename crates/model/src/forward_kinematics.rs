use crate::configuration::{Configuration, ConfigurationError};
use crate::data::{Data, PyData};
use crate::model::{Model, PyModel, WORLD_FRAME_ID};
use joint::joint::JointWrapper;
use nalgebra::IsometryMatrix3;
use numpy::PyReadonlyArray1;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

// TODO: make this a method of the Model struct
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
        return Err(ConfigurationError::InvalidSize("q".to_string(), model.nq, q.len()));
    }

    // update the joints data
    let mut offset = 0;
    let mut keys: Vec<_> = data.joint_data.keys().cloned().collect();
    keys.sort();
    for id in keys {
        let joint_data = data.joint_data.get_mut(&id).unwrap();
        let joint_model: Box<&JointWrapper> = Box::new(model.joint_models.get(&id).unwrap());
        let q_joint = q.rows(offset, joint_model.nq()).into_owned();
        match joint_data.update(&joint_model, &q_joint) {
            Ok(_) => {}
            Err(e) => {
                return Err(ConfigurationError::JointDataUpdateError(id, e));
            }
        }
        offset += joint_model.nq();
    }

    // update the placements of the joints in the world frame
    // by traversing the joint tree
    data.joint_placements.clear();
    data.joint_placements
        .insert(WORLD_FRAME_ID, IsometryMatrix3::identity());

    let mut keys = model.joint_models.keys().collect::<Vec<_>>();
    keys.sort();
    for joint_id in keys {
        let parent_id = model.joint_parents.get(joint_id).unwrap(); // we checked that the parent existed before
        // get the placement of the parent join in the world frame
        let parent_placement = data.joint_placements.get(parent_id).unwrap();
        // get the placement of the joint in the parent frame
        let local_joint_placement = model.joint_placements.get(joint_id).unwrap();
        // get the joint transformation
        let joint_data = data.joint_data.get(joint_id).unwrap();
        let joint_placement = joint_data.get_joint_placement();
        // compute the placement of the joint in the world frame
        data.joint_placements.insert(
            *joint_id,
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
