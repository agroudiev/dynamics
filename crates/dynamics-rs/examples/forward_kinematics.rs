use dynamics_model::forward_kinematics::update_frame_placements;
use dynamics_model::model::random_configuration;
use dynamics_rs::model::forward_kinematics::forward_kinematics;
use dynamics_rs::parse::urdf::build_models_from_urdf;

fn main() {
    let urdf_path = "./examples/descriptions/ur5/ur5_robot.urdf"; // Path to the URDF file
    let mesh_path = "./examples/descriptions/ur5/"; // Optional mesh path

    // Build models from the URDF file
    let (model, _coll_model, _viz_model) =
        build_models_from_urdf(urdf_path, Some(mesh_path)).expect("Failed to parse URDF file");

    // Generate a random configuration
    let q = random_configuration(&model);
    println!("Random configuration q: {}", q);

    // Create data structure for the model
    let mut data = model.create_data();

    // Compute forward kinematics
    forward_kinematics(&model, &mut data, &q, None, None)
        .expect("Failed to compute forward kinematics");

    // Print the placement of the joint 'wrist_3_joint'
    let id = model.get_joint_id("wrist_3_joint").unwrap();
    let placement = &data.joint_placements[id];
    println!("Placement of 'wrist_3_joint':\n{:?}", placement);

    // Compute the frame placements
    update_frame_placements(&model, &mut data);

    // Print the placement of the frame 'tool0'
    let frame_id = model.get_frame_id("tool0", None).unwrap();
    // we don't specify a frame type (None) as there is only one frame with this name
    let frame_placement = &data.frame_placements[frame_id];
    println!("Placement of frame 'tool0':\n{:?}", frame_placement);
}
