use crate::urdf::build_models_from_urdf;
use collider::shape::Cylinder;
use nalgebra::{IsometryMatrix3, Translation3};

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
    assert_eq!(model.name, "materials");
    assert_eq!(geom_model.models.len(), 3);

    assert_eq!(*model.frames.get(&0).unwrap(), IsometryMatrix3::identity());
    assert_eq!(
        model.frames.get(&1).unwrap().translation,
        Translation3::new(0.0, -0.22, 0.25)
    );
    assert_eq!(
        model.frames.get(&2).unwrap().translation,
        Translation3::new(0.0, 0.22, 0.25)
    );

    assert_eq!(geom_model.models.get(&1).unwrap().parent_frame, 1);
    assert_eq!(geom_model.models.get(&2).unwrap().parent_frame, 2);

    let data = model.create_data();
    let geom_data = geom_model.create_data(&model, &data);
    assert_eq!(
        geom_data.get_object_placement(1).unwrap().translation,
        Translation3::new(0.0, -0.22, 0.25 - 0.3)
    );
    assert_eq!(
        geom_data.get_object_placement(2).unwrap().translation,
        Translation3::new(0.0, 0.22, 0.25 - 0.3)
    );
}

#[test]
fn test_double_pendulum_simple() {
    let filepath = "../../examples/descriptions/double_pendulum_simple.urdf";
    let result = build_models_from_urdf(filepath);
    let (model, _geom_model) = result.unwrap();
    assert_eq!(model.name, "2dof_planar");
}
