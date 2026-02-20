use crate::{
    data::{Data, GeometryData},
    py_geometry_model::PyGeometryModel,
    py_model::PyModel,
};
use dynamics_inertia::py_inertia::PyInertia;
use dynamics_joint::py_joint_data::PyJointDataWrapper;
use dynamics_spatial::{
    py_configuration::PyConfiguration, py_force::PySpatialForce, py_jacobian::PyJacobian,
    py_motion::PySpatialMotion, py_se3::PySE3,
};
use pyo3::prelude::*;

/// Structure containing the mutable properties of the robot.
#[pyclass(name = "Data")]
pub struct PyData {
    pub inner: Data,
}

#[pymethods]
impl PyData {
    #[new]
    /// Creates a new `Data` object.
    ///
    /// # Arguments
    ///
    /// * `model` - The model object.
    ///
    /// # Returns
    /// A new `Data` object corresponding to the given model.
    #[must_use]
    pub fn new(model: &PyModel) -> Self {
        PyData {
            inner: model.inner.create_data(),
        }
    }

    #[getter]
    /// Returns the placements of the joints in the world frame.
    #[must_use]
    pub fn joint_placements(&self) -> Vec<PySE3> {
        self.inner
            .joint_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    #[getter]
    /// Returns the placements of the frames in the world frame.
    #[must_use]
    pub fn frame_placements(&self) -> Vec<PySE3> {
        self.inner
            .frame_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the placements of the joints in the world frame.
    ///
    /// This is an alias for `joint_placements` to match the Pinocchio API.
    #[must_use]
    pub fn oMi(&self) -> Vec<PySE3> {
        self.joint_placements()
    }

    /// The local joint placements in the parent frame (liMi)
    #[getter]
    pub fn local_joint_placements(&self) -> Vec<PySE3> {
        self.inner
            .local_joint_placements
            .iter()
            .map(|p| PySE3 { inner: *p })
            .collect()
    }

    /// The local joint placements in the parent frame (liMi)
    #[getter]
    #[allow(non_snake_case)]
    pub fn liMi(&self) -> Vec<PySE3> {
        self.local_joint_placements()
    }

    #[getter]
    #[allow(non_snake_case)]
    /// Returns the placements of the frames in the world frame.
    ///
    /// This is an alias for `frame_placements` to match the Pinocchio API.
    #[must_use]
    pub fn oMf(&self) -> Vec<PySE3> {
        self.frame_placements()
    }

    #[getter]
    pub fn joints(&self) -> Vec<PyJointDataWrapper> {
        self.inner
            .joint_data
            .iter()
            .map(|jd| PyJointDataWrapper { inner: jd.clone() })
            .collect()
    }

    #[getter]
    /// Returns the joint velocities.
    pub fn joint_velocities(&self) -> Vec<PySpatialMotion> {
        self.inner
            .joint_velocities
            .iter()
            .map(|m| PySpatialMotion { inner: m.clone() })
            .collect()
    }

    #[getter]
    /// Returns the joint velocities.
    ///
    /// This is an alias for `joint_velocities` to match the Pinocchio API.
    pub fn v(&self) -> Vec<PySpatialMotion> {
        self.joint_velocities()
    }

    #[getter]
    /// Returns the joint accelerations.
    pub fn joint_accelerations(&self) -> Vec<PySpatialMotion> {
        self.inner
            .joint_accelerations
            .iter()
            .map(|m| PySpatialMotion { inner: m.clone() })
            .collect()
    }

    #[getter]
    /// Returns the joint accelerations.
    ///
    /// This is an alias for `joint_accelerations` to match the Pinocchio API.
    pub fn a(&self) -> Vec<PySpatialMotion> {
        self.joint_accelerations()
    }

    #[getter]
    /// Accelerations of the joints due to the gravity field (a_gf)
    pub fn joint_accelerations_gravity_field(&self) -> Vec<PySpatialMotion> {
        self.inner
            .joint_accelerations_gravity_field
            .iter()
            .map(|m| PySpatialMotion { inner: m.clone() })
            .collect()
    }

    #[getter]
    /// Accelerations of the joints due to the gravity field (a_gf)
    pub fn a_gf(&self) -> Vec<PySpatialMotion> {
        self.joint_accelerations_gravity_field()
    }

    #[getter]
    /// Accelerations of the joints in the world frame including the gravity field (oa_gf)
    pub fn world_accelerations_gravity_field(&self) -> Vec<PySpatialMotion> {
        self.inner
            .world_accelerations_gravity_field
            .iter()
            .map(|m| PySpatialMotion { inner: m.clone() })
            .collect()
    }

    #[getter]
    /// Accelerations of the joints in the world frame including the gravity field (oa_gf)
    pub fn oa_gf(&self) -> Vec<PySpatialMotion> {
        self.world_accelerations_gravity_field()
    }

    #[getter]
    /// The spatial momenta of the joint in the local frame (h), inertia times velocity
    pub fn joint_momenta(&self) -> Vec<PySpatialForce> {
        self.inner
            .joint_momenta
            .iter()
            .map(|f| PySpatialForce { inner: f.clone() })
            .collect()
    }

    #[getter]
    /// The spatial momenta of the joint in the local frame (h), inertia times velocity
    pub fn h(&self) -> Vec<PySpatialForce> {
        self.joint_momenta()
    }

    #[getter]
    /// The spatial momenta of the joint in the world frame (oh), inertia times velocity
    pub fn world_joint_momenta(&self) -> Vec<PySpatialForce> {
        self.inner
            .world_joint_momenta
            .iter()
            .map(|f| PySpatialForce { inner: f.clone() })
            .collect()
    }

    #[getter]
    /// The spatial momenta of the joint in the world frame (oh), inertia times velocity
    pub fn oh(&self) -> Vec<PySpatialForce> {
        self.world_joint_momenta()
    }

    #[getter]
    /// The spatial forces of the joint in the local frame (f), inertia times acceleration plus the Coriolis term
    pub fn joint_forces(&self) -> Vec<PySpatialForce> {
        self.inner
            .joint_forces
            .iter()
            .map(|f| PySpatialForce { inner: f.clone() })
            .collect()
    }

    #[getter]
    /// The spatial forces of the joint in the local frame (f), inertia times acceleration plus the Coriolis term
    pub fn f(&self) -> Vec<PySpatialForce> {
        self.joint_forces()
    }

    #[getter]
    /// The spatial forces of the joint in the world frame (of), inertia times acceleration plus the Coriolis term
    pub fn world_joint_forces(&self) -> Vec<PySpatialForce> {
        self.inner
            .world_joint_forces
            .iter()
            .map(|f| PySpatialForce { inner: f.clone() })
            .collect()
    }

    #[getter]
    /// The spatial forces of the joint in the world frame (of), inertia times acceleration plus the Coriolis term
    pub fn of(&self) -> Vec<PySpatialForce> {
        self.world_joint_forces()
    }

    #[getter]
    /// The configuration of torques/forces applied to the joints (tau)
    pub fn tau(&self) -> PyConfiguration {
        PyConfiguration(self.inner.tau.clone())
    }

    #[getter]
    /// The joint accelerations of the joints computed by the forward dynamics (ddq)
    pub fn ddq(&self) -> PyConfiguration {
        PyConfiguration(self.inner.ddq.clone())
    }

    #[getter]
    /// The Jacobian matrix of the joint placements (J)
    pub fn jacobian(&self) -> PyJacobian {
        PyJacobian {
            inner: self.inner.jacobian.clone(),
        }
    }

    #[allow(non_snake_case)]
    /// The Jacobian matrix of the joint placements (J)
    #[getter]
    pub fn J(&self) -> PyJacobian {
        self.jacobian()
    }

    #[getter]
    /// The rigid body inertia of the joints in the world frame (oinertias)
    pub fn world_inertias(&self) -> Vec<PyInertia> {
        self.inner
            .world_inertias
            .iter()
            .map(|i| PyInertia { inner: i.clone() })
            .collect()
    }

    #[getter]
    /// The rigid body inertia of the joints in the world frame (oinertias)
    pub fn oinertias(&self) -> Vec<PyInertia> {
        self.world_inertias()
    }

    #[getter]
    /// The composite rigid body inertia of the joints in the world frame (oYcrb)
    pub fn composite_inertias(&self) -> Vec<PyInertia> {
        self.inner
            .composite_inertias
            .iter()
            .map(|i| PyInertia { inner: i.clone() })
            .collect()
    }

    #[getter]
    /// The composite rigid body inertia of the joints in the world frame (oYcrb)
    #[allow(non_snake_case)]
    pub fn oYcrb(&self) -> Vec<PyInertia> {
        self.composite_inertias()
    }
}

/// Structure containing the mutable geometric data of the models.
#[pyclass(name = "GeometryData")]
pub struct PyGeometryData {
    pub inner: GeometryData,
}

#[pymethods]
impl PyGeometryData {
    #[new]
    /// Creates a new `GeometryData` object.
    ///
    /// # Arguments
    ///
    /// * `model` - The model object.
    /// * `data` - The data object.
    /// * `geom_model` - The geometry model object.
    ///
    /// # Returns
    /// A new `GeometryData` object.
    #[must_use]
    pub fn new(data: &PyData, geom_model: &PyGeometryModel) -> Self {
        let mut geom_data = GeometryData::default();
        geom_data.update_geometry_data(&data.inner, &geom_model.inner);
        PyGeometryData { inner: geom_data }
    }

    /// Returns the placement of the object of given index in the world frame.
    ///
    /// # Arguments
    ///
    /// * `object_index` - The index of the object.
    ///
    /// # Returns
    /// The object placement if it exists, otherwise `None`.
    pub fn get_object_placement(&self, object_index: usize) -> PyResult<PySE3> {
        match self.inner.get_object_placement(object_index) {
            Some(placement) => Ok(PySE3 { inner: *placement }),
            None => Err(pyo3::exceptions::PyKeyError::new_err(format!(
                "Object with index {object_index} not found"
            ))),
        }
    }

    /// Updates the geometry data using the updated model data and geometry model.
    ///
    /// # Arguments
    ///
    /// * `data` - The model data object.
    /// * `geom_model` - The geometry model object.
    pub fn update_geometry_data(&mut self, data: &PyData, geom_model: &PyGeometryModel) {
        self.inner
            .update_geometry_data(&data.inner, &geom_model.inner);
    }
}
