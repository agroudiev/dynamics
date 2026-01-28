use dynamics_model::py_geometry_model::PyGeometryModel;
use dynamics_model::py_model::PyModel;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::urdf::build_models_from_urdf;

/// A Python wrapper for the `build_models_from_urdf` function.
#[pyfunction(name = "build_models_from_urdf")]
#[pyo3(signature = (filepath, package_dir = None))]
pub fn py_build_models_from_urdf(
    filepath: &str,
    package_dir: Option<&str>,
) -> PyResult<(PyModel, PyGeometryModel, PyGeometryModel)> {
    match build_models_from_urdf(filepath, package_dir) {
        Ok((model, coll_model, viz_model)) => Ok((
            PyModel { inner: model },
            PyGeometryModel { inner: coll_model },
            PyGeometryModel { inner: viz_model },
        )),
        Err(e) => Err(PyErr::new::<PyValueError, _>(format!("{e}"))),
    }
}
