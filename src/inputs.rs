use crate::controller::SetPoints;
use crate::na::Vector3;
use crate::util::{clamp, map_range};
use gilrs::{Axis, Button, Event, EventType, Gilrs};

// Units =
const MAX_LONG_FORCE: f32 = 10.0;

// Units = degrees/second
// Converted to radians locally
const MAX_ANGULAR_VEL_X: f32 = 40.0;
const MAX_ANGULAR_VEL_Y: f32 = 40.0;
const MAX_ANGULAR_VEL_Z: f32 = 40.0;

pub struct Inputs {
    ctx: Gilrs,
    pub set_points: SetPoints,
    pub aux: AuxControls,
}

// TODO reset/etc
// view_mode, look-at/follow/etc, buttons
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Default)]
pub struct AuxControls {
    pub reset_all: bool,
    pub view_mode: ViewMode,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum ViewMode {
    Static,
    LookAt,
    Follow,
}

impl Default for ViewMode {
    fn default() -> Self {
        ViewMode::Static
    }
}

impl Inputs {
    pub fn new() -> Self {
        Inputs {
            ctx: Gilrs::new().unwrap(),
            set_points: SetPoints {
                angular_velocity: Vector3::zeros(),
                longitudinal_force: 0.0,
            },
            aux: AuxControls::default(),
        }
    }

    pub fn update(&mut self) {
        if let Some(Event { id, event, time: _ }) = self.ctx.next_event() {
            let input = self.ctx.gamepad(id);

            self.aux.reset_all = input.is_pressed(Button::South);

            match event {
                EventType::ButtonPressed(Button::West, _) => {
                    self.aux.view_mode = match self.aux.view_mode {
                        ViewMode::Static => ViewMode::LookAt,
                        ViewMode::LookAt => ViewMode::Follow,
                        ViewMode::Follow => ViewMode::Static,
                    };
                    println!("view mode {:?}", self.aux.view_mode);
                }
                _ => (),
            }

            let f_neg = if let Some(btn) = input.button_data(Button::LeftTrigger2) {
                btn.value()
            } else {
                0.0
            };

            let f_pos = if let Some(btn) = input.button_data(Button::RightTrigger2) {
                btn.value()
            } else {
                0.0
            };

            self.set_points.longitudinal_force = map_range(
                (-1.0, 1.0),
                (-MAX_LONG_FORCE, MAX_LONG_FORCE),
                f_pos - f_neg,
            );

            self.set_points.angular_velocity.x = map_range(
                (-1.0, 1.0),
                (
                    -MAX_ANGULAR_VEL_X.to_radians(),
                    MAX_ANGULAR_VEL_X.to_radians(),
                ),
                input.value(Axis::RightStickX),
            );

            self.set_points.angular_velocity.y = map_range(
                (-1.0, 1.0),
                (
                    MAX_ANGULAR_VEL_Y.to_radians(),
                    -MAX_ANGULAR_VEL_Y.to_radians(),
                ),
                input.value(Axis::LeftStickX),
            );

            self.set_points.angular_velocity.z = map_range(
                (-1.0, 1.0),
                (
                    MAX_ANGULAR_VEL_Z.to_radians(),
                    -MAX_ANGULAR_VEL_Z.to_radians(),
                ),
                input.value(Axis::RightStickY),
            );
        }
    }
}
