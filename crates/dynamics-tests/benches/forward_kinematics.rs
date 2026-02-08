use criterion::{Criterion, criterion_group, criterion_main};
use dynamics_rs::prelude::*;
use dynamics_tests::utils::EXAMPLE_ROBOT_DATA_URDFS;
use std::hint::black_box;

fn test_forward_kinematics(model: &Model) {
    let mut data = model.create_data();

    let q = random_configuration(model);
    let v = Configuration::from_element(model.nv, 1.0);
    let a = Configuration::from_element(model.nv, 1.0);

    forward_kinematics(model, &mut data, &q, Some(&v), Some(&a)).unwrap();
}

fn bench_forward_kinematics(c: &mut Criterion) {
    // set ROS_PACKAGE_PATH to find the example-robot-data package
    unsafe {
        std::env::set_var(
            "ROS_PACKAGE_PATH",
            "../../examples/descriptions/example-robot-data:".to_string()
                + &std::env::var("ROS_PACKAGE_PATH").unwrap_or_default(),
        );
    }

    c.bench_function("bench_forward_kinematics", |b| {
        let mut models = Vec::new();
        for path in EXAMPLE_ROBOT_DATA_URDFS {
            let (model, _, _) = build_models_from_urdf(
                &("../../examples/descriptions/example-robot-data/robots/".to_string() + path),
                None,
            )
            .unwrap();
            models.push(model);
        }
        b.iter(|| {
            for model in &models {
                black_box(test_forward_kinematics)(model);
            }
        });
    });
}

criterion_group!(benches, bench_forward_kinematics,);
criterion_main!(benches);
