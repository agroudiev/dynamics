import dynamics
import collider
import meshcat

geometries = [
    collider.Capsule(0.1, 0.8),
    collider.Sphere(0.5),
    collider.Cuboid((1, 1, 1)),
    collider.Cylinder(0.1, 1.0),
    collider.Cone(0.5, 1.0),
]