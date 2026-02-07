use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use dynamics_tests::utils::EXAMPLE_ROBOT_DATA_URDFS;

use dynamics_rs::prelude::*;

fn test_parse_urdf(path: &str) {
    build_models_from_urdf(
        &("../../examples/descriptions/example-robot-data/robots/".to_string() + path),
        None,
    )
    .unwrap();
}

fn bench_parse_urdf(c: &mut Criterion) {
    // set ROS_PACKAGE_PATH to find the example-robot-data package
    unsafe {
        std::env::set_var(
            "ROS_PACKAGE_PATH",
            "../../examples/descriptions/example-robot-data:".to_string()
                + &std::env::var("ROS_PACKAGE_PATH").unwrap_or_default(),
        );
    }

    c.bench_function("bench_parse_urdf", |b| {
        b.iter(|| {
            for path in EXAMPLE_ROBOT_DATA_URDFS {
                black_box(test_parse_urdf)(path)
            }
        });
    });
}

criterion_group!(benches, bench_parse_urdf,);
criterion_main!(benches);
