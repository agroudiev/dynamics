//! This crate exports the bindings for the `dynamics` crate to Python.
//!
//! Note that it also re-exports the `collider` bindings as the Python
//! `dynamics.collider` submodule. For technical reasons, only this submodule
//! should be used to interact between `collider` and `dynamics`. In the future,
//! this may be changed (using for instance serialization of the shapes).

use inertia::inertia::PyInertia;
use joint::revolute::{PyJointModelRevolute, new_joint_model_revolute_x};
use model::{
    data::{PyData, PyGeometryData},
    forward_dynamics::{py_aba, py_forward_dynamics},
    forward_kinematics::py_forward_kinematics,
    geometry_model::PyGeometryModel,
    geometry_object::PyGeometryObject,
    inverse_dynamics::{py_inverse_dynamics, py_rnea},
    model::py_random_configuration,
    model::{PyModel, STANDARD_GRAVITY, WORLD_FRAME_ID},
    neutral::py_neutral,
};
use pyo3::prelude::*;

use collider::mesh::PyMesh;
use collider::shape::{
    PyShapeWrapper, ShapeType, capsule::PyCapsule, cone::PyCone, cuboid::PyCuboid,
    cylinder::PyCylinder, sphere::PySphere,
};
use numpy::PyArray1;
use parse::urdf::py_build_models_from_urdf;
use spatial::se3::PySE3;

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
    dynamics.add_class::<PyModel>()?;
    dynamics.add_class::<PyData>()?;
    dynamics.add_class::<PyGeometryModel>()?;
    dynamics.add_class::<PyGeometryData>()?;
    dynamics.add_class::<PyGeometryObject>()?;
    dynamics.add_function(wrap_pyfunction!(py_neutral, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_random_configuration, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_build_models_from_urdf, dynamics)?)?;

    dynamics.add("WORLD_FRAME_ID", WORLD_FRAME_ID)?;
    dynamics.add(
        "STANDARD_GRAVITY",
        PyArray1::from_slice(py, STANDARD_GRAVITY.data.as_slice()),
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

    py.import("sys")?
        .getattr("modules")?
        .set_item("dynamics.SE3", se3)?;

    Ok(())
}

fn add_joint_bindings(dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    dynamics.add_class::<PyJointModelRevolute>()?;
    dynamics.add_function(wrap_pyfunction!(new_joint_model_revolute_x, dynamics)?)?;

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

    dynamics.add_function(wrap_pyfunction!(py_forward_dynamics, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_aba, dynamics)?)?;

    dynamics.add_function(wrap_pyfunction!(py_inverse_dynamics, dynamics)?)?;
    dynamics.add_function(wrap_pyfunction!(py_rnea, dynamics)?)?;

    Ok(())
}
