#![allow(unused_variables)]

#[macro_use]
extern crate nalgebra as na;
extern crate ahrs;

#[cfg(test)]
mod test{

  use ahrs::AHRS;
  use na::{Vec3,Quat,ApproxEq};
  //use na;

  #[test]
  fn test_equal_quats() {
      assert_approx_eq!( &Quat::new(1.0f64, 2.0, 3.0, 4.0), &Quat::new(1.0f64, 2.0, 3.0, 4.0) );
  }

  #[test]
  fn test_update_known_vals() {

    let mut ahrs = AHRS::default();
    let accel = Vec3::new(-3.102632460623745e-02f64, 3.049616351072091e-02, 9.721632321637426e-01);
    let gyro = Vec3::new(0.0f64, 0.0, 0.0); //Vec3::new(0.056f64, 0.8740, -0.4510);
    let mag = Vec3::new(1.4f64, 1.4, 1.2);

    for i in 1i64..2000 {
        ahrs.UpdateIMU(gyro, accel);
    }

    //println!("computed : {}\t{}\t{}\t{}", ahrs.Quaternion[0], ahrs.Quaternion[1], ahrs.Quaternion[2], ahrs.Quaternion[3]);
    //println!("expected : 0.999750\t-0.015666\t-0.015939\t-2.661844e-18");
    
    assert_approx_eq!( &ahrs.Quaternion, &Quat::new( 0.999750219795400f64,
                                                        -0.015666683681890,
                                                        -0.015939041422266,
                                                        -2.661844518290388e-18
                                                       ) );
  }

}

