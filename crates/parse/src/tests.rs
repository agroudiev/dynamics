#[cfg(test)]
mod tests {
    use crate::urdf::build_models_from_urdf;
    use collider::shape::Cylinder;

    #[test]
    fn test_myfirst() {
        let filepath = "../../examples/descriptions/myfirst.urdf";
        let result = build_models_from_urdf(filepath);
        let (model, geom_model) = result.unwrap();
        assert_eq!(model.name, "myfirst");

        assert_eq!(geom_model.models.len(), 1);
        let object = geom_model.models.get(0).unwrap();
        assert_eq!(object.name, "base_link");
        assert_eq!(object.geometry.as_ref(), &Cylinder::new(0.2, 0.3));
    }

    #[test]
    #[ignore]
    fn test_double_pendulum_simple() {
        let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
        let result = build_models_from_urdf(filepath);
        let (model, _geom_model) = result.unwrap();
        assert_eq!(model.name, "2dof_planar");
    }
}
