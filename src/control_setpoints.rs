use crate::na::geometry::UnitQuaternion;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;

// TODO - simple force + angular/torque then switch to attitude/velocity
// controller
pub struct ControlSetpoints {
    //velocity: Velocity<f32>,
    //attitude: UnitQuaternion<f32>,
    pub ag_force: f32,
    pub long_force: f32,
    pub lat_force: f32,
}

impl ControlSetpoints {
    pub fn new() -> Self {
        ControlSetpoints {
            ag_force: 0.0,
            long_force: 0.0,
            lat_force: 0.0,
        }
    }
}
