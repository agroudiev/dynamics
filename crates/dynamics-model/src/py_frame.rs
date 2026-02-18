use dynamics_inertia::py_inertia::PyInertia;
use dynamics_spatial::py_se3::PySE3;
use pyo3::prelude::*;

use crate::frame::{Frame, FrameType};

/// A coordinate frame attached to a specific location of the robot.
///
/// Frames can be attached to joints, bodies, or arbitrary points on the robot.
/// They contain information about their placement relative to their parent joint or frame,
/// as well as any associated inertia.
///
/// Each frame is uniquely identified by its name and type.
/// Two frames of different types can share the same name.
#[pyclass(name = "Frame")]
#[derive(Clone, Debug)]
pub struct PyFrame {
    pub inner: Frame,
}

#[pymethods]
impl PyFrame {
    #[new]
    #[must_use]
    /// Creates a new Frame.
    ///
    /// # Arguments
    /// * `name` - Name of the frame.
    /// * `parent_joint` - Index of the parent joint the frame is attached to.
    /// * `parent_frame` - Index of the parent frame in the model's frames vector.
    /// * `placement` - Placement of the frame with respect to the parent frame.
    /// * `frame_type` - Type of the frame.
    /// * `inertia` - Inertia associated to the frame.
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
    /// Returns the name of the frame.
    pub fn name(&self) -> &String {
        &self.inner.name
    }

    #[getter]
    #[must_use]
    /// Returns the index of the parent joint the frame is attached to.
    pub fn parent_joint(&self) -> usize {
        self.inner.parent_joint
    }

    #[getter]
    #[must_use]
    /// Returns the index of the parent frame in the model's frames vector.
    pub fn parent_frame(&self) -> usize {
        self.inner.parent_frame
    }

    #[getter]
    #[must_use]
    /// Returns the type of the frame.
    pub fn frame_type(&self) -> FrameType {
        self.inner.frame_type.clone()
    }

    #[getter]
    #[must_use]
    /// Returns the placement of the frame with respect to the parent frame as an SE(3) transformation.
    pub fn placement(&self) -> PySE3 {
        PySE3 {
            inner: self.inner.placement,
        }
    }

    #[getter]
    #[must_use]
    /// Returns the inertia associated to the frame.
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
