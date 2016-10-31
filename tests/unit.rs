#![allow(unused_variables, dead_code)]

#[macro_use]
extern crate nalgebra as na;
extern crate ahrs;

//#[cfg(test)]
//mod test{

  use ahrs::Ahrs;
  use na::{Vector3, Quaternion, approx_eq};
  use std::f64;

  #[test]
  fn test_update_known_vals() {

    let start_quat = Quaternion::new( 0.7252997863255918f64,
                                      0.6869689552600526,
                                     -0.04486780259245286,
                                      0.0008687666471569602);

    let mut ahrs = Ahrs::default();
    ahrs.quat = start_quat;

    let accel = Vector3::new(0.06640625, 0.9794922, -0.01269531);
    let gyro = Vector3::new(68.75, 34.25, 3.0625);
    let mag = Vector3::new(0.171875, -0.4536133, -0.04101563);

    ahrs.update(gyro * (f64::consts::PI/180.0), accel, mag);

    let expected = Quaternion::new( 0.7235467139148768,
                                    0.6888611247479446,
                                   -0.04412605927634125,
                                    0.001842413287185898);

    let fail_message = format!("quaternions did not match:\n\
          actual: {:?}\n\
          expect: {:?}", ahrs.quat, expected);

    assert!(approx_eq(&ahrs.quat, &expected), fail_message);
  }

  #[test]
  fn test_update_imu_known_vals() {

    let start_quat = Quaternion::new( 0.7208922848226422,
                                      0.6922487447935516,
                                     -0.01829063767755937,
                                      0.02777483732249482);

    let mut ahrs = Ahrs::default();
    ahrs.quat = start_quat;

    let accel = Vector3::new(0.06640625, 0.9794922, -0.01269531);
    let gyro = Vector3::new(68.75, 34.25, 3.0625);
    let mag = Vector3::new(0.171875, -0.4536133, -0.04101563);

    ahrs.update_imu(gyro * (f64::consts::PI/180.0), accel);

    let expected = Quaternion::new( 0.7190919791549198,
                                    0.694101991692336,
                                   -0.01747200330433749,
                                    0.02870330545992814);

    let fail_message = format!("quaternions did not match:\n\
        actual: {:?}\n\
        expect: {:?}", ahrs.quat, expected);

    assert!(approx_eq(&ahrs.quat, &expected), fail_message);
  }

//}

