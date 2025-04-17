#[cfg(test)]
mod tests {
    use crate::urdf::build_models_from_urdf;

    #[test]
    fn test_myfirst() {
        let filepath = "../../examples/descriptions/myfirst.urdf";
        let result = build_models_from_urdf(filepath);
        let (model, geom_model) = result.unwrap();

        assert_eq!(model.name, "myfirst");
    }

    #[test]
    fn test_double_pendulum_simple() {
        let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
        let result = build_models_from_urdf(filepath);
        let (model, geom_model) = result.unwrap();

        assert_eq!(model.name, "2dof_planar");
    }
}
