# dynamics
An experimental implementation of Rigid Body Dynamics algorithms.

## API
The API is designed to be close to the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library. This project is not a direct port of Pinocchio, but rather an experimental attempt to create a similar API in Rust with Python bindings. The goal is to provide a high-level interface for rigid body dynamics, while also allowing for low-level access to the underlying algorithms. Examples of the Python API can be found in the [`examples/python`](https://github.com/agroudiev/dynamics/tree/main/examples/python) directory.

Please note that this project is still in its early stages and is not recommended for production use. The API may change significantly in the future as the project evolves.

## Python dependencies
Along with classical dependencies like `numpy`, this project uses:
- [`collider`](https://github.com/agroudiev/collider) for collision detection
- [`meshcat`](https://github.com/meshcat-dev/meshcat-python/) for visualization
Support for other visualization libraries might be added in the future.
