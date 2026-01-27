//! Coordinate frames attached to joints.

use dynamics_inertia::inertia::Inertia;
use dynamics_inertia::py_inertia::PyInertia;
use dynamics_spatial::se3::{PySE3, SE3};
use pyo3::prelude::*;

#[derive(Clone, Debug, PartialEq, Eq)]
#[pyclass]
pub enum FrameType {
    /// Operational frames for task space control.
    Operational,
    /// Frames directly associated to joints.
    Joint,
    /// Frames for fixed joints
    Fixed,
    /// Frames attached to robot bodies.
    Body,
    /// Frames for sensor locations.
    Sensor,
}

#[derive(Clone, Debug)]
pub struct Frame {
    /// Name of the frame.
    pub name: String,
    /// Index of the parent joint the frame is attached to.
    pub parent_joint: usize,
    /// Index of the parent frame in the model's frames vector.
    pub parent_frame: usize,
    /// Type of the frame.
    pub frame_type: FrameType,
    /// Placement of the frame with respect to the parent frame.
    pub placement: SE3,
    /// Inertia associated to the frame.
    pub inertia: Inertia,
}

impl Frame {
    /// Creates a new Frame.
    #[must_use]
    pub fn new(
        name: String,
        parent_joint: usize,
        parent_frame: usize,
        placement: SE3,
        frame_type: FrameType,
        inertia: Inertia,
    ) -> Self {
        Frame {
            name,
            parent_joint,
            parent_frame,
            frame_type,
            placement,
            inertia,
        }
    }
}

#[pyclass(name = "Frame")]
#[derive(Clone, Debug)]
pub struct PyFrame {
    pub inner: Frame,
}

#[pymethods]
impl PyFrame {
    #[new]
    #[must_use]
    pub fn new(
        name: String,
        parent_joint: usize,
        parent_frame: usize,
        placement: PySE3,
        frame_type: FrameType,
        inertia: PyInertia,
    ) -> Self {
        PyFrame {
            inner: Frame::new(
                name,
                parent_joint,
                parent_frame,
                placement.inner,
                frame_type,
                inertia.inner,
            ),
        }
    }

    #[getter]
    #[must_use]
    pub fn name(&self) -> String {
        self.inner.name.clone()
    }

    #[getter]
    #[must_use]
    pub fn parent_joint(&self) -> usize {
        self.inner.parent_joint
    }

    #[getter]
    #[must_use]
    pub fn parent_frame(&self) -> usize {
        self.inner.parent_frame
    }

    #[getter]
    #[must_use]
    pub fn frame_type(&self) -> FrameType {
        self.inner.frame_type.clone()
    }

    #[getter]
    #[must_use]
    pub fn placement(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.placement,
        }
    }

    #[getter]
    #[must_use]
    pub fn inertia(&self) -> PyInertia {
        PyInertia {
            inner: self.inner.inertia.clone(),
        }
    }

    #[must_use]
    pub fn __repr__(&self) -> String {
        format!(
            "Frame (name='{}', parent joint={}, parent frame={}, frame_type={:?})\n Placement={:?}\n Inertia={:?}",
            self.inner.name,
            self.inner.parent_joint,
            self.inner.parent_frame,
            self.inner.frame_type,
            self.inner.placement,
            self.inner.inertia
        )
    }
}
