use crate::util::{clamp, map_range};
use gilrs::{Axis, Button, Event, Gilrs};
use nphysics3d::math::Force;
//use nphysics3d::math::Velocity;
//use crate::na::geometry::UnitQuaternion;

// TODO - simple force + angular/torque then switch to attitude/velocity
// controller
pub struct ControlSetpoints {
    //velocity: Velocity<f32>,
    //attitude: UnitQuaternion<f32>,
    pub ag_force: f32,
    pub long_force: f32,
    pub lat_force: f32,
    pub rot: f32,
}

impl ControlSetpoints {
    pub fn new() -> Self {
        ControlSetpoints {
            ag_force: 0.0,
            long_force: 0.0,
            lat_force: 0.0,
            rot: 0.0,
        }
    }
}

pub struct AuxControls {
    //pub view_mode look-at/follow/etc
}

pub struct Controller {
    ctx: Gilrs,
    setpoints: ControlSetpoints,
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            ctx: Gilrs::new().unwrap(),
            setpoints: ControlSetpoints::new(),
        }
    }

    pub fn update(&mut self) -> &ControlSetpoints {
        let mut f_up = 0.0;
        let mut f_down = 0.0;

        if let Some(Event { id, event, time: _ }) = self.ctx.next_event() {
            let input = self.ctx.gamepad(id);

            if let Some(btn) = input.button_data(Button::LeftTrigger2) {
                f_down = btn.value();
            }
            if let Some(btn) = input.button_data(Button::RightTrigger2) {
                f_up = btn.value();
            }

            let f_long = input.value(Axis::RightStickY);
            let f_lat = input.value(Axis::RightStickX);

            self.setpoints.rot = input.value(Axis::LeftStickX);

            self.setpoints.ag_force = map_range((-1.0, 1.0), (-60.0, 60.0), f_up - f_down);

            self.setpoints.long_force = map_range((-1.0, 1.0), (-10.0, 10.0), f_long);

            self.setpoints.lat_force = map_range((-1.0, 1.0), (-10.0, 10.0), f_lat);

            //println!("{:?}", event);
            //println!("ag {}", self.setpoints.ag_force);
        }

        &self.setpoints
    }
}
