#[cfg(test)]
mod tests {
    use crate::urdf::build_model_from_urdf;

    #[test]
    fn test_urdf_parser() {
        let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
        let result = build_model_from_urdf(filepath);
        assert!(result.is_ok());
    }
}
