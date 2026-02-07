use approx::assert_relative_eq;
use dynamics_rs::prelude::*;

#[ignore]
#[test]
fn test_rnea_ur5() {
    let path = "../../examples/descriptions/ur5/ur5_robot.urdf";
    let package_dir = "../../examples/descriptions/ur5/";
    let (model, _, _) = build_models_from_urdf(path, Some(package_dir)).unwrap();
    let mut data = model.create_data();

    let q = Configuration::from_row_slice(&[
        6.11532514,
        -0.07569121,
        -1.4693574,
        -5.14300212,
        5.62677151,
        -5.3564271,
    ]);
    let v = Configuration::from_row_slice(&[
        0.56538218, 0.28044434, 0.98835702, 0.10830778, 0.72461398, 0.44884571,
    ]);
    let a = Configuration::from_row_slice(&[
        0.58728518, 0.63471225, 0.94532479, 0.63963593, 0.82708475, 0.13476001,
    ]);

    inverse_dynamics(&model, &mut data, &q, &v, &a, None).unwrap();
    let expected_tau = Configuration::from_row_slice(&[
        2.19845646e+00,
        -4.03839257e+01,
        7.27140560e-01,
        4.86391603e-01,
        -5.31880719e-03,
        4.12162545e-02,
    ]);
    assert_relative_eq!(data.tau, expected_tau);
}
