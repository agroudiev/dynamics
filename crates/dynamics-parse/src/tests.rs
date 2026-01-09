use crate::urdf::build_models_from_urdf;
use collider::shape::Cylinder;
use model::model::WORLD_FRAME_ID;
use spatial::{motion::SpatialRotation, se3::SE3, vector3d::Vector3D};

#[test]
fn test_myfirst() {
    let filepath = "../../examples/descriptions/myfirst.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();
    assert_eq!(model.name, "myfirst");

    assert_eq!(geom_model.models.len(), 1);
    let object = geom_model.models.get(&0).unwrap();
    assert_eq!(object.name, "base_link");
    assert_eq!(object.geometry.as_ref(), &Cylinder::new(0.2, 0.3));
}

// TODO: test all shapes

#[test]
fn test_multipleshapes() {
    let filepath = "../../examples/descriptions/multipleshapes.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();
    assert_eq!(model.name, "multipleshapes");
    assert_eq!(geom_model.models.len(), 2);
}

#[test]
fn test_origins() {
    let filepath = "../../examples/descriptions/origins.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();
    assert_eq!(model.name, "origins");
    assert_eq!(geom_model.models.len(), 2);

    // TODO: test placement
}

#[test]
fn test_materials() {
    let filepath = "../../examples/descriptions/materials.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();

    let data = model.create_data();
    let geom_data = geom_model.create_data(&data);

    assert_eq!(model.name, "materials");
    assert_eq!(geom_model.models.len(), 3);

    // base link
    assert_eq!(model.joint_placements[WORLD_FRAME_ID], SE3::identity());
    assert_eq!(*geom_data.get_object_placement(0).unwrap(), SE3::identity());

    dbg!(&model.joint_names);
    dbg!(&model.joint_parents);

    // right leg
    let base_right_leg_id = model.get_joint_id("base_to_right_leg").unwrap();
    let right_leg_id = model.get_joint_id("right_leg").unwrap();
    assert_eq!(
        model.joint_placements[base_right_leg_id].translation(),
        Vector3D::new(0.0, -0.22, 0.25)
    );
    // assert_eq!(geom_model.models.get(&right_leg_id).unwrap().parent_joint, right_leg_id);
    assert_eq!(
        geom_data
            .get_object_placement(right_leg_id)
            .unwrap()
            .translation(),
        Vector3D::new(0.0, -0.22, 0.25 - 0.3)
    );

    // left leg
    let base_left_leg_id = model.get_joint_id("base_to_left_leg").unwrap();
    let left_leg_id = model.get_joint_id("left_leg").unwrap();
    assert_eq!(
        model.joint_placements[base_left_leg_id].translation(),
        Vector3D::new(0.0, 0.22, 0.25)
    );
    // assert_eq!(geom_model.models.get(&left_leg_id).unwrap().parent_joint, left_leg_id);
    assert_eq!(
        geom_data
            .get_object_placement(left_leg_id)
            .unwrap()
            .translation(),
        Vector3D::new(0.0, 0.22, 0.25 - 0.3)
    );
}

#[test]
fn test_visuals() {
    let filepath = "../../examples/descriptions/visuals.urdf";
    let (model, geom_model) = build_models_from_urdf(filepath).unwrap();
    assert_eq!(model.name, "visual");

    let data = model.create_data();
    let geom_data = geom_model.create_data(&data);

    let box_id = model.get_joint_id("box").unwrap();
    assert_eq!(
        geom_data
            .get_object_placement(box_id)
            .unwrap()
            .translation(),
        Vector3D::new(0.1814, 0.0, 0.1414) + Vector3D::new(0.0, 0.0, 0.3)
    );

    let right_base_id = geom_model.indices.get("right_base").unwrap();
    assert_eq!(
        geom_data
            .get_object_placement(*right_base_id)
            .unwrap()
            .rotation(),
        SpatialRotation::identity()
    );
    assert_eq!(
        geom_data
            .get_object_placement(*right_base_id)
            .unwrap()
            .translation(),
        Vector3D::new(0.0, 0.0, -0.6) + Vector3D::new(0.0, -0.22, 0.25)
    );
}

#[test]
fn test_double_pendulum_simple() {
    let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();
    assert_eq!(model.name, "2dof_planar");

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
}

#[test]
fn test_ur5() {
    let filepath = "../../examples/descriptions/ur5/ur5_robot.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, geom_model) = result.unwrap();
    assert_eq!(model.name, "ur5");

    let data = model.create_data();
    let _geom_data = geom_model.create_data(&data);
}
