//! Vector math and stuff

use nalgebra::{Quaternion, UnitQuaternion, UnitVector3, Vector3};

/// Get rotation unit quaternion from IMU. Rotations are clockwise.
///
/// You can't directly apply IMU readings to a rotation matrix and expect accurate
/// results; remember that inbetween each step of the rotation matrix the axis
/// changes. Also not to mention the axes are always moving with the IMU.
///
/// The IMU simply is measuring a *single* axis-angle rotation with 3 primitive
/// sensors, so we need to convert the data read by the sensors into 1 angle axis
/// rotation.
pub fn multi_rotate(
    x: f32,
    y: f32,
    z: f32,
    x_axis: UnitQuaternion<f32>,
    y_axis: UnitQuaternion<f32>,
    z_axis: UnitQuaternion<f32>,
) -> UnitQuaternion<f32> {
    // Honestly don't ask me for a detailed explination. This came to me in a
    // dream about rotating spheres in my head. Not sure how mathmatically
    // correct it is, but it works well afaik.
    let x_vec = Vector3::from(x_axis.vector()) * x;
    let y_vec = Vector3::from(y_axis.vector()) * y;
    let z_vec = Vector3::from(z_axis.vector()) * z;

    let rotation = x_vec + y_vec + z_vec;

    let axis = UnitVector3::new_normalize(rotation);

    let angle = rotation.magnitude();

    let rotation_quaternion = UnitQuaternion::from_axis_angle(&axis, angle);

    rotation_quaternion
}

/// Get the initial axes unit quaternions from initial measured accelerations.
pub fn initial_unit_quaternions(
    accelerations: Vector3<f32>,
) -> (
    UnitQuaternion<f32>,
    UnitQuaternion<f32>,
    UnitQuaternion<f32>,
) {
    let gravity_vector =
        UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, accelerations));

    let x_unit =
        UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, Vector3::new(1.0, 0.0, 0.0)));
    let y_unit =
        UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, Vector3::new(0.0, 1.0, 0.0)));
    let z_unit =
        UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, Vector3::new(0.0, 0.0, 1.0)));

    // Our gravity vector represents our true "Z" axis. In order to align our
    // acceleration vectors, we first find the rotation to make gravity aligned
    // to the Z axis, and apply that rotation to all of the unit quaternions
    let rotation_quaternion = gravity_vector.rotation_to(&z_unit);

    let x_unit = rotation_quaternion * x_unit;
    let y_unit = rotation_quaternion * y_unit;
    let z_unit = rotation_quaternion * z_unit;

    (x_unit, y_unit, z_unit)
}
