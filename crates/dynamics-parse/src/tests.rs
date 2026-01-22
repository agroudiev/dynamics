use crate::urdf::build_models_from_urdf;

#[test]
fn test_myfirst() {
    let filepath = "../../examples/descriptions/myfirst.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, coll_model, viz_model) = result.unwrap();
    assert_eq!(model.name, "myfirst");

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

// TODO: test all shapes

#[test]
fn test_multipleshapes() {
    let filepath = "../../examples/descriptions/multipleshapes.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, coll_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_origins() {
    let filepath = "../../examples/descriptions/origins.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, geom_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_materials() {
    let filepath = "../../examples/descriptions/materials.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, coll_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_visuals() {
    let filepath = "../../examples/descriptions/visuals.urdf";
    let (model, coll_model, viz_model) = build_models_from_urdf(filepath, None).unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_double_pendulum_simple() {
    let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, coll_model, viz_model) = result.unwrap();
    assert_eq!(model.name, "2dof_planar");

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_ur5_classical() {
    let filepath = "../../examples/descriptions/ur5/ur5_robot.urdf";
    let result = build_models_from_urdf(filepath, Some("../../examples/descriptions/ur5"));
    let (model, coll_model, viz_model) = result.unwrap();
    assert_eq!(model.name, "ur5");

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_ur5_example_robot_data() {
    // set ROS_PACKAGE_PATH to find the example-robot-data package
    unsafe {
        std::env::set_var(
            "ROS_PACKAGE_PATH",
            "../../examples/descriptions/example-robot-data:".to_string()
                + &std::env::var("ROS_PACKAGE_PATH").unwrap_or_default(),
        );
    }
    // specifying the path here with ../.. is a bit ugly,
    // but it is necessary to make the test work when run from the root

    let filepath =
        "../../examples/descriptions/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf";
    let result = build_models_from_urdf(filepath, None);
    let (model, coll_model, viz_model) = result.unwrap();
    assert_eq!(model.name, "ur5");

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}
