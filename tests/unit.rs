#![allow(unused_variables, dead_code)]

#[macro_use]
extern crate nalgebra as na;
extern crate ahrs;

#[cfg(test)]
mod test{

  use ahrs::Ahrs;
  use na::{Vector3, Quaternion, approx_eq};

  #[test]
  fn test_equal_quats() {
      assert!( approx_eq(&Quaternion::new(1.0f64, 2.0, 3.0, 4.0), &Quaternion::new(1.0f64, 2.0, 3.0, 4.0)) );
  }

  //#[test]
  fn test_update_known_vals() {

    let mut ahrs = Ahrs::default();
    let accel = Vector3::new(-3.102632460623745e-02f64, 3.049616351072091e-02, 9.721632321637426e-01);
    let gyro = Vector3::new(0.0f64, 0.0, 0.0);
    let mag = Vector3::new(1.4f64, 1.4, 1.2);

    for i in 1i64..2000 {
        ahrs.update_imu(gyro, accel);
    }

    assert!( approx_eq(&ahrs.quat, &Quaternion::new( 0.999750219795400f64,
                                                    -0.015666683681890,
                                                    -0.015939041422266,
                                                    -2.661844518290388e-18
                                                   ) ));
  }

}

