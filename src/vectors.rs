//! Vector math and stuff

use ape_table_trig::{abs_f32, PI_F32};
use nalgebra::{Vector3, Quaternion, UnitQuaternion, UnitVector3};

/// Get rotation unit quaternion from IMU. Rotations are clockwise.
pub fn multi_rotate(x: f32, y: f32, z: f32) -> UnitQuaternion<f32> {
    let rotation = Vector3::new(x, y, z);

    let axis = UnitVector3::new_normalize(rotation);

    let x = abs_f32(x);
    let y = abs_f32(y);
    let z = abs_f32(z);

    let rot_sum = x + y + z;

    let x_weight = x / rot_sum;
    let y_weight = y / rot_sum;
    let z_weight = z / rot_sum;

    let angle = x * x_weight + y * y_weight + z * z_weight;

    let rotation_quaternion = UnitQuaternion::from_axis_angle(&axis, angle);

    rotation_quaternion
}

/// Get the initial axes unit quaternions
pub fn initial_unit_quaternions(accelerations: Vector3<f32>) -> (UnitQuaternion<f32>, UnitQuaternion<f32>, UnitQuaternion<f32>) {
    // The Z quaternion is just a unit verison of the acceleraions, and can be used to get the
    // other vectors
    let z_unit = UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, accelerations));

    // In order to get the X quaternion we will rotate the Z quaternion 90 degrees counterclockwise
    // along the Y
    let x_rotate = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.5 * PI_F32);
    let x_unit = x_rotate.inverse() * z_unit * x_rotate;

    // In order to get the Y quaternion we will rotate the Z quaternion 90 degrees clockwise
    // along the X
    let y_rotate = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.5 * PI_F32);
    let y_unit = y_rotate.inverse() * z_unit * y_rotate;
    
    (x_unit, y_unit, z_unit)
}
