import dynamics as dyn

# load an URDF file
model, coll_model, viz_model = dyn.build_models_from_urdf(
    "examples/descriptions/double_pendulum_simple.urdf"
)

# create a visualizer
viz = dyn.visualize.MeshcatVisualizer(model, coll_model, viz_model)
viz.init_viewer(load_model=True)

viz.open()
input("Press enter to continue...")
