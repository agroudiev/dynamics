use dynamics_parse::urdf::build_models_from_urdf;

fn main() {
    let filepath = "examples/descriptions/origins.urdf";
    let result = build_models_from_urdf(filepath);
    println!("{:?}", result.unwrap());
}
