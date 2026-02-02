use dynamics_rs::parse::urdf::build_models_from_urdf;

fn main() {
    let urdf_path = "./examples/descriptions/ur5/ur5_robot.urdf"; // Path to the URDF file
    let mesh_path = "./examples/descriptions/ur5/"; // Optional mesh path

    // Build models from the URDF file
    let (model, _coll_model, _viz_model) =
        build_models_from_urdf(urdf_path, Some(mesh_path)).expect("Failed to parse URDF file");

    // Print the names of the joints
    println!("Joints in the model:");
    for joint_name in &model.joint_names {
        println!("- {}", joint_name);
    }

    // Print the names of the frames
    println!("\nFrames in the model:");
    for frame in &model.frames {
        println!("- {}", frame.name);
    }
    println!();

    // Print the joint tree
    model.print_joint_tree().unwrap();
}
