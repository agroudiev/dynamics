from . import dynamics
import meshcat

class MeshcatVisualizer:
    """A dynamics visualizer using meshcat."""
    def __init__(
        self,
        model=dynamics.Model(),
        collision_model=None,
        visual_model=None,
    ):
        # Check if the arguments are valid
        if collision_model is not None:
            assert isinstance(collision_model, dynamics.GeometryModel), \
                "collision_model must be a GeometryModel"
        if visual_model is not None:
            assert isinstance(visual_model, dynamics.GeometryModel), \
                "visual_model must be a GeometryModel"
        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

    def init_viewer(self, viewer=None, open=False, load_model=False, url=None):
        """Start a new meschat server and client."""

        self.viewer = meshcat.Visualizer(url) if viewer is None else viewer

        if open:
            self.viewer.open()

        if load_model:
            self.load_model()

    def load_model(self):
        print("Warning: load_model is not implemented yet.")

    def reset(self):
        self.viewer.delete()