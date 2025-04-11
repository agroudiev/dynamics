//! This crate exports the bindings for the `dynamics` crate to Python.
//!
//! Note that it also re-exports the `collider` bindings as the Python
//! `dynamics.collider` submodule. For technical reasons, only this submodule
//! should be used to interact between `collider` and `dynamics`. In the future,
//! this may be changed (using for instance serialization of the shapes).

use model::{geometry_object::PyGeometryObject, model::PyModel, geometry_model::PyGeometryModel};
use pyo3::prelude::*;

use collider::shape::{
    capsule::PyCapsule, cone::PyCone, cuboid::PyCuboid, cylinder::PyCylinder, sphere::PySphere,
};
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
    dynamics.add_class::<PyGeometryModel>()?;
    dynamics.add_class::<PyGeometryObject>()?;

    add_spatial_bindings(py, dynamics)?;

    Ok(())
}

fn add_spatial_bindings(py: Python, dynamics: &Bound<'_, PyModule>) -> PyResult<()> {
    let se3 = PyModule::new(dynamics.py(), "SE3")?;
    se3.add_function(wrap_pyfunction!(spatial::se3::identity, &se3)?)?;
    dynamics.add_submodule(&se3)?;

    // adding the SE3 class between add_submodule and setting sys.modules
    // to avoid name conflicts
    dynamics.add_class::<PySE3>()?;

    py.import("sys")?
        .getattr("modules")?
        .set_item("dynamics.SE3", se3)?;

    Ok(())
}

fn add_shapes_bindings(collider: &Bound<'_, PyModule>) -> PyResult<()> {
    collider.add_class::<PyCapsule>()?;
    collider.add_class::<PyCone>()?;
    collider.add_class::<PyCuboid>()?;
    collider.add_class::<PyCylinder>()?;
    collider.add_class::<PySphere>()?;

    Ok(())
}
