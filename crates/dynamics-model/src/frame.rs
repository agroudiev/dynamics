//! Coordinate frames attached to joints.

use dynamics_inertia::inertia::Inertia;
use dynamics_spatial::se3::SE3;
use pyo3::prelude::*;

#[derive(Clone, Debug)]
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
    pub fn new(
        name: String,
        parent_joint: usize,
        parent_frame: usize,
        frame_type: FrameType,
        placement: SE3,
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
pub struct PyFrame {
    pub inner: Frame,
}

#[pymethods]
impl PyFrame {
    #[getter]
    pub fn name(&self) -> String {
        self.inner.name.clone()
    }
}
