use numpy::{ToPyArray, ndarray::Array2};
use pyo3::prelude::*;

use crate::jacobian::Jacobian;

#[pyclass(name = "Jacobian")]
#[derive(Debug, Clone, PartialEq)]
pub struct PyJacobian {
    pub inner: Jacobian,
}

#[pymethods]
impl PyJacobian {
    #[new]
    pub fn new(cols: usize) -> Self {
        PyJacobian {
            inner: Jacobian::zero(cols),
        }
    }

    pub fn to_numpy(&self, py: Python) -> Py<PyAny> {
        let mat = &self.inner.0;
        Array2::from_shape_fn((6, mat.ncols()), |(i, j)| mat[(i, j)])
            .to_pyarray(py)
            .into_any()
            .unbind()
    }

    pub fn update_column(&mut self, v_offset: usize, column_data: [f64; 6]) {
        self.inner.update_column(v_offset, &column_data);
    }
}
