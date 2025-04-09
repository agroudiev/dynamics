use pyo3::prelude::*;

/// A Python module implemented in Rust.
#[pymodule(name = "dynamics")]
fn dynamics_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    Ok(())
}
