import dynamics as dyn

# Load a robot model from a URDF file
model, _coll_model, _viz_model = dyn.build_models_from_urdf(
    "./examples/descriptions/ur5/ur5_robot.urdf",
    "./examples/descriptions/ur5/",  # mesh directory
)
data = model.create_data()

# Define a random configuration
q = dyn.random_configuration(model)
print(f"Random configuration q: {q}")

# Compute forward kinematics
dyn.forward_kinematics(model, data, q)

# Print the placement of a specific joint and frame
joint_id = model.get_joint_id("wrist_3_joint")
frame_id = model.get_frame_id("tool0")  # don't specify the type

print(f"Joint {joint_id} placement:\n{data.joint_placements[joint_id]}")
print(f"Frame {frame_id} placement:\n{data.frame_placements[frame_id]}")
