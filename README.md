# dynamics
An experimental implementation of Rigid Body Dynamics algorithms.
<div align="center">
  <a href="https://crates.io/crates/dynamics-rs">
    <img src="https://img.shields.io/crates/v/dynamics-rs.svg" alt="crates.io Latest Release"/>
  </a>
  <a href="https://pypi.org/project/rdynamics-py/">
    <img src="https://img.shields.io/pypi/v/rdynamics-py.svg" alt="PyPi Latest Release"/>
  </a>
  <a href="https://docs.rs/dynamics-rs/latest/dynamics_rs/">
    <img src="https://img.shields.io/docsrs/dynamics-rs" alt="Docs.rs Documentation"/>
  </a>
</div>

## API
The API is designed to be close to the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library. This project is not a direct port of Pinocchio, but rather an experimental attempt to create a similar API in Rust. The goal is to provide a high-level interface for rigid body dynamics, while also allowing for low-level access to the underlying algorithms. Examples of the Python API can be found in the [`examples/python`](https://github.com/agroudiev/dynamics/tree/main/examples/python) directory.

Please note that this project is still in its early stages and is not recommended for production use. The API may change significantly in the future as the project evolves.

## Python dependencies
Along with classical dependencies like `numpy`, this project uses:
- [`collider`](https://github.com/agroudiev/collider) for collision detection
- [`meshcat`](https://github.com/meshcat-dev/meshcat-python/) for visualization
Support for other visualization libraries might be added in the future.

## Installation
This project is implemented using both Rust and Python. The Python bindings are created using [PyO3](https://pyo3.rs/), and [maturin](https://www.maturin.rs/) as the build system.

The use of [miniconda](https://docs.conda.io/en/latest/miniconda.html) is recommended to manage the dependencies. To install the dependencies, run the following command:
```bash
conda env create -f dynamics_env.yml
```
To activate the environment, run:
```bash
conda activate dynamics
```

[maturin](https://www.maturin.rs/) can be installed directly using `pip`:
```bash
pip install maturin
```
To build the Rust code and install it directly as a Python package in the current `dynamics` virtual environment, run:
```bash
maturin develop --release -m dynamics-py/Cargo.toml
```
To run unit test of all crates, run:
```bash
cargo test
```