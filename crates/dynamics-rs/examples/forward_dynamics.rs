use dynamics_rs::prelude::*;

fn main() {
    let urdf_path = "./examples/descriptions/ur5/ur5_robot.urdf"; // Path to the URDF file
    let mesh_path = "./examples/descriptions/ur5/"; // Optional mesh path

    // Build models from the URDF file
    let (model, _coll_model, _viz_model) =
        build_models_from_urdf(urdf_path, Some(mesh_path)).expect("Failed to parse URDF file");

    // Generate a random configuration, velocity and acceleration
    let q = random_configuration(&model);
    let v = Configuration::from_element(model.nv, 1.0);
    let tau = Configuration::from_element(model.nv, 1.0);

    // Create data structure for the model
    let mut data = model.create_data();

    // Compute forward dynamics
    let ddq = forward_dynamics(&model, &mut data, &q, &v, &tau, None, ABAConvention::Local)
        .expect("Failed to compute forward dynamics");

    // Print the computed torques
    println!("Joint accelerations: {}", ddq);
}
