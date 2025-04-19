//! Model for a geometry object, that is a shape or mesh with visualization properties.

use collider::shape::*;
use nalgebra::{IsometryMatrix3, Vector4};
use numpy::{IntoPyArray, PyReadonlyArray1, ndarray::Array1};
use pyo3::{exceptions::PyValueError, prelude::*, types::PyTuple};
use spatial::se3::PySE3;

/// A `GeometryObject` is a data structure that contains the information about the geometry object,
/// used for visualization, collision detection and distance computation.
pub struct GeometryObject {
    /// The name of the geometry object.
    pub name: String,
    /// Whether to disable collision detection and distance check for this object.
    pub disable_collision: bool,
    /// The `collider` geometry object.
    pub geometry: ShapeWrapper,
    /// The RGBA color of the mesh.
    pub mesh_color: Vector4<f64>,
    /// The placement of the geometry object in the parent frame.
    pub placement: IsometryMatrix3<f64>,
}

impl GeometryObject {
    /// Creates a new `GeometryObject` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the geometry object.
    /// * `parent_joint` - The identifier of the parent joint.
    /// * `parent_frame` - The identifier of the parent frame.
    /// * `geometry` - The `collider` shape of the geometry object (used for collisions).
    /// * `mesh_color` - The RGBA color of the mesh.
    /// * `placement` - The placement of the geometry object in the parent frame.
    pub fn new(
        name: String,
        _parent_joint: usize,
        _parent_frame: usize,
        geometry: ShapeWrapper,
        mesh_color: Vector4<f64>,
        placement: IsometryMatrix3<f64>,
    ) -> Self {
        Self {
            name,
            disable_collision: false,
            geometry,
            mesh_color,
            placement,
        }
    }
}

impl Clone for GeometryObject {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            disable_collision: self.disable_collision,
            geometry: self.geometry.clone_box(),
            mesh_color: self.mesh_color,
            placement: self.placement,
        }
    }
}

/// A `GeometryObject` is a data structure that contains the information about the geometry object,
/// used for visualization, collision detection and distance computation.
#[pyclass(name = "GeometryObject")]
pub struct PyGeometryObject {
    pub inner: GeometryObject,
}

#[pymethods]
impl PyGeometryObject {
    /// Creates a new `GeometryObject` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the geometry object.
    /// * `parent_joint` - The identifier of the parent joint.
    /// * `parent_frame` - The identifier of the parent frame.
    /// * `geometry` - The `collider` shape of the geometry object (used for collisions).
    /// * `placement` - The placement of the geometry object in the parent frame.
    #[new]
    #[pyo3(signature = (*py_args))]
    fn new(py_args: &Bound<'_, PyTuple>) -> PyResult<Self> {
        if py_args.len() == 5 {
            let name: String = py_args.get_item(0)?.extract()?;
            let _parent_joint: usize = py_args.get_item(1)?.extract()?;
            let _parent_frame: usize = py_args.get_item(2)?.extract()?;

            let geometry: PyShapeWrapper =
                if let Ok(capsule) = py_args.get_item(3)?.extract::<PyRef<PyCapsule>>() {
                    PyShapeWrapper {
                        inner: capsule.inner.clone_box(),
                    }
                } else if let Ok(cone) = py_args.get_item(3)?.extract::<PyRef<PyCone>>() {
                    PyShapeWrapper {
                        inner: cone.inner.clone_box(),
                    }
                } else if let Ok(cuboid) = py_args.get_item(3)?.extract::<PyRef<PyCuboid>>() {
                    PyShapeWrapper {
                        inner: cuboid.inner.clone_box(),
                    }
                } else if let Ok(cylinder) = py_args.get_item(3)?.extract::<PyRef<PyCylinder>>() {
                    PyShapeWrapper {
                        inner: cylinder.inner.clone_box(),
                    }
                } else if let Ok(sphere) = py_args.get_item(3)?.extract::<PyRef<PySphere>>() {
                    PyShapeWrapper {
                        inner: sphere.inner.clone_box(),
                    }
                } else {
                    return Err(PyValueError::new_err(format!(
                        "expected a shape for 'geometry', but got {:?}.",
                        py_args.get_item(3)?.get_type()
                    )));
                };

            let placement = py_args.get_item(4)?.extract::<PyRef<PySE3>>()?;

            Ok(Self {
                inner: GeometryObject::new(
                    name,
                    _parent_joint,
                    _parent_frame,
                    geometry.inner.clone_box(),
                    Vector4::new(0.0, 0.0, 0.0, 1.0),
                    placement.inner,
                ),
            })
        } else {
            Err(PyValueError::new_err(format!(
                "incorrect number of arguments, expected 5, but got {}. Signature: GeometryObject(name, parent_joint, parent_frame, geometry, placement)",
                py_args.len()
            )))
        }
    }

    #[setter]
    fn set_mesh_color(&mut self, color: PyReadonlyArray1<f64>) -> PyResult<()> {
        let color = color.as_array();
        if color.len() == 4 {
            let r: f64 = color[0];
            let g: f64 = color[1];
            let b: f64 = color[2];
            let a: f64 = color[3];
            self.inner.mesh_color = Vector4::new(r, g, b, a);
            Ok(())
        } else {
            Err(PyValueError::new_err(format!(
                "expected a NumPy array of length 4 for 'color', but got {}.",
                color.len()
            )))
        }
    }

    #[getter]
    fn get_mesh_color(&self, py: Python) -> Py<PyAny> {
        Array1::from_shape_vec(4, self.inner.mesh_color.as_slice().to_vec())
            .unwrap()
            .into_pyarray(py)
            .into_any()
            .unbind()
    }

    #[getter]
    fn get_geometry(&self) -> PyShapeWrapper {
        PyShapeWrapper {
            inner: self.inner.geometry.clone_box(),
        }
    }

    #[getter]
    fn get_name(&self) -> String {
        self.inner.name.clone()
    }

    #[getter]
    fn get_placement(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.placement,
        }
    }
}
