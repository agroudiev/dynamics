use crate::urdf::build_models_from_urdf;

#[test]
fn test_myfirst() {
    let filepath = "../../examples/descriptions/myfirst.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, _, _) = result.unwrap();
    assert_eq!(model.name, "myfirst");
}

// TODO: test all shapes

#[test]
fn test_multipleshapes() {
    let filepath = "../../examples/descriptions/multipleshapes.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, coll_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_origins() {
    let filepath = "../../examples/descriptions/origins.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_materials() {
    let filepath = "../../examples/descriptions/materials.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, coll_model, viz_model) = result.unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_visuals() {
    let filepath = "../../examples/descriptions/visuals.urdf";
    let (model, coll_model, viz_model) = build_models_from_urdf(filepath).unwrap();

    let data = model.create_data();
    let _coll_data = coll_model.create_data(&data);
    let _viz_data = viz_model.create_data(&data);
}

#[test]
fn test_double_pendulum_simple() {
    let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model, _) = result.unwrap();
    assert_eq!(model.name, "2dof_planar");

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
}

#[test]
fn test_ur5() {
    let filepath = "../../examples/descriptions/ur5/ur5_robot.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model, _) = result.unwrap();
    assert_eq!(model.name, "ur5");

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
}
