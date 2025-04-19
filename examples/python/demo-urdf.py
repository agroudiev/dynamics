import dynamics as dyn

# load an URDF file
model, geom_model = dyn.build_models_from_urdf("examples/descriptions/origins.urdf")

# create a visualizer
viz = dyn.visualize.MeshcatVisualizer(model, geom_model, geom_model)
viz.init_viewer(load_model=True)

viz.open()
input("Press enter to continue...")