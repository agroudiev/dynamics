//! This crate exports the bindings for the `dynamics` crate to Python.
//!
//! Note that it also re-exports the `collider` bindings as the Python
//! `dynamics.collider` submodule. For technical reasons, only this submodule
//! should be used to interact between `collider` and `dynamics`. In the future,
//! this may be changed (using for instance serialization of the shapes).

use dynamics_inertia::py_inertia::PyInertia;
use dynamics_joint::{
    joint::JointType,
    py_joint::{
        PyJointWrapper, new_px, new_py, new_pz, new_revolute_unaligned, new_rubx, new_ruby,
        new_rubz, new_rx, new_ry, new_rz,
    },
    py_joint_data::PyJointDataWrapper,
};
use dynamics_model::{
    frame::FrameType,
    model::{STANDARD_GRAVITY, WORLD_ID},
    py_algorithms::{
        py_aba, py_forward_dynamics, py_forward_kinematics, py_inverse_dynamics, py_neutral,
        py_rnea, py_update_frame_placements,
    },
    py_data::{PyData, PyGeometryData},
    py_frame::PyFrame,
    py_geometry_model::PyGeometryModel,
    py_geometry_object::PyGeometryObject,
    py_model::{PyModel, py_random_configuration},
};
use pyo3::prelude::*;

use collider_rs::{
    mesh::py_mesh::PyMesh,
    shape::{
        ShapeType, py_capsule::PyCapsule, py_cone::PyCone, py_cuboid::PyCuboid,
        py_cylinder::PyCylinder, py_shape::PyShapeWrapper, py_sphere::PySphere,
    },
};
use dynamics_parse::py_urdf::py_build_models_from_urdf;
use dynamics_spatial::{
    py_configuration::PyConfiguration, py_force::PySpatialForce, py_motion::PySpatialMotion,
    py_se3::PySE3, py_symmetric3::PySymmetric3, py_vector3d::PyVector3D, py_vector6d::PyVector6D,
};
use numpy::PyArray1;

#[pymodule(name = "dynamics")]
fn dynamics_py(py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    add_dynamics_bindings(py, dynamics)?;
    collider_py(py, dynamics)?;

    Ok(())
}

fn collider_py(py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    // Create a new module for the re-exported collider bindings
    let collider = PyModule::new(dynamics.py(), "collider")?;

    // Add the bindings to the module
    add_shapes_bindings(&collider)?;

    // Add it as a submodule, and link it so it can be imported as `dynamics.collider`
    dynamics.add_submodule(&collider)?;
    py.import("sys")?
        .getattr("modules")?
        .set_item("dynamics.collider", collider)?;

    Ok(())
}

fn add_dynamics_bindings(py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    dynamics.add_class::<PyConfiguration>()?;
    dynamics.add_class::<PyModel>()?;
    dynamics.add_class::<PyData>()?;
    dynamics.add_class::<PyFrame>()?;
    dynamics.add_class::<FrameType>()?;
    dynamics.add_class::<PyGeometryModel>()?;
    dynamics.add_class::<PyGeometryData>()?;
    dynamics.add_class::<PyGeometryObject>()?;
    dynamics.add_function(wrap_pyfunction!(py_neutral, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_random_configuration, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_build_models_from_urdf, dynamics)?)?;

    dynamics.add("WORLD_ID", WORLD_ID)?;
    dynamics.add(
        "STANDARD_GRAVITY",
        PyArray1::from_slice(py, STANDARD_GRAVITY.as_slice()),
    )?;

    add_inertia_bindings(dynamics)?;
    add_joint_bindings(dynamics)?;
    add_spatial_bindings(py, dynamics)?;
    add_algorithms_bindings(py, dynamics)?;

    Ok(())
}

fn add_spatial_bindings(py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    let se3 = PyModule::new(dynamics.py(), "SE3")?;
    dynamics.add_submodule(&se3)?;

    // adding the SE3 class between add_submodule and setting sys.modules
    // to avoid name conflicts
    dynamics.add_class::<PySE3>()?;
    dynamics.add_class::<PyVector3D>()?;
    dynamics.add_class::<PyVector6D>()?;
    dynamics.add_class::<PySymmetric3>()?;
    dynamics.add_class::<PySpatialMotion>()?;
    dynamics.add_class::<PySpatialForce>()?;

    py.import("sys")?
        .getattr("modules")?
        .set_item("dynamics.SE3", se3)?;

    Ok(())
}

fn add_joint_bindings(dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    dynamics.add_class::<JointType>()?;
    dynamics.add_class::<PyJointWrapper>()?;
    dynamics.add_class::<PyJointDataWrapper>()?;

    dynamics.add_function(wrap_pyfunction!(new_rx, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_ry, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_rz, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_revolute_unaligned, dynamics)?)?;

    dynamics.add_function(wrap_pyfunction!(new_rubx, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_ruby, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_rubz, dynamics)?)?;

    dynamics.add_function(wrap_pyfunction!(new_px, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_py, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(new_pz, dynamics)?)?;

    Ok(())
}

fn add_inertia_bindings(dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    dynamics.add_class::<PyInertia>()?;

    Ok(())
}

fn add_shapes_bindings(collider: &Bound<'_, PyModule>) -> PyResult<()> {
    collider.add_class::<PyCapsule>()?;
    collider.add_class::<PyCone>()?;
    collider.add_class::<PyCuboid>()?;
    collider.add_class::<PyCylinder>()?;
    collider.add_class::<PySphere>()?;
    collider.add_class::<PyMesh>()?;

    collider.add_class::<PyShapeWrapper>()?;
    collider.add_class::<ShapeType>()?;

    Ok(())
}

fn add_algorithms_bindings(_py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    dynamics.add_function(wrap_pyfunction!(py_forward_kinematics, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_update_frame_placements, dynamics)?)?;

    dynamics.add_function(wrap_pyfunction!(py_forward_dynamics, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_aba, dynamics)?)?;

    dynamics.add_function(wrap_pyfunction!(py_inverse_dynamics, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_rnea, dynamics)?)?;

    Ok(())
}
