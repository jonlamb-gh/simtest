// TODO - tune the PIDs
// like here https://www.crossco.com/blog/basics-tuning-pid-loops

use crate::na::geometry::UnitQuaternion;
use crate::na::{Isometry3, Matrix3, Point3, Vector3};
use crate::util::{clamp, map_range};
use gilrs::{Axis, Button, Event, Gilrs};
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use pid::Pid;

pub struct Setpoints {
    pub ag_vel: f32,
    pub long_accel: f32,
    pub yaw_rate: f32,
    pub pitch_rate: f32,
    /*velocity: Velocity<f32>,
     *attitude: UnitQuaternion<f32>,
     *    pub ag_force: f32,
     *    pub long_force: f32,
     *    pub lat_force: f32,
     *    pub rot: f32, */
}

impl Setpoints {
    pub fn new() -> Self {
        Setpoints {
            ag_vel: 0.0,
            long_accel: 0.0,
            yaw_rate: 0.0,
            pitch_rate: 0.0,
        }
    }
}

pub struct Inputs {
    pub rot: UnitQuaternion<f32>,
    // Local frame
    pub vel: Velocity<f32>,
}

pub struct Outputs {
    // World frame
    pub ag_force: Force<f32>,

    // Local frame
    pub long_force: Force<f32>,
    // Angular
    pub pseudo_torque: Vector3<f32>,
}

impl Outputs {
    pub fn new() -> Self {
        Outputs {
            ag_force: Force::zero(),
            long_force: Force::zero(),
            pseudo_torque: Vector3::zeros(),
        }
    }
}

// reset/etc
pub struct AuxControls {
    //pub view_mode look-at/follow/etc, buttons
}

pub struct Controller {
    ctx: Gilrs,
    pid_vert_vel: Pid<f32>,
    pid_yaw_rate: Pid<f32>,
    pid_pitch_rate: Pid<f32>,
    setpoints: Setpoints,
    outputs: Outputs,
}

impl Controller {
    pub fn new() -> Self {
        let kp = 100.0;
        let ki = 0.0;
        let kd = 0.0;
        let p_limit = 100.0;
        let i_limit = 10.0;
        let d_limit = 1.0;
        let mut pid_vert_vel = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_vert_vel.update_setpoint(0.0);

        let kp = 5.0;
        let ki = 0.0;
        let kd = 0.0;
        let p_limit = 5.0;
        let i_limit = 1.0;
        let d_limit = 1.0;
        let mut pid_yaw_rate = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_yaw_rate.update_setpoint(0.0);

        let kp = 5.0;
        let ki = 0.0;
        let kd = 0.0;
        let p_limit = 5.0;
        let i_limit = 1.0;
        let d_limit = 1.0;
        let mut pid_pitch_rate = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_pitch_rate.update_setpoint(0.0);

        Controller {
            ctx: Gilrs::new().unwrap(),
            setpoints: Setpoints::new(),
            outputs: Outputs::new(),
            pid_vert_vel,
            pid_yaw_rate,
            pid_pitch_rate,
        }
    }

    pub fn update(&mut self, inputs: &Inputs) -> &Outputs {
        if let Some(Event {
            id,
            event: _,
            time: _,
        }) = self.ctx.next_event()
        {
            let mut f_up = 0.0;
            let mut f_down = 0.0;

            let input = self.ctx.gamepad(id);

            if let Some(btn) = input.button_data(Button::LeftTrigger2) {
                f_down = btn.value();
            }
            if let Some(btn) = input.button_data(Button::RightTrigger2) {
                f_up = btn.value();
            }

            let f_long = input.value(Axis::LeftStickY);
            let f_ry = input.value(Axis::LeftStickX);
            let f_rz = input.value(Axis::RightStickY);

            self.setpoints.ag_vel = map_range((-1.0, 1.0), (-2.0, 2.0), f_up - f_down);

            self.setpoints.long_accel = map_range((-1.0, 1.0), (-5.0, 5.0), f_long);

            self.setpoints.yaw_rate = map_range((-1.0, 1.0), (-0.5, 0.5), -f_ry);

            self.setpoints.pitch_rate = map_range((-1.0, 1.0), (-0.5, 0.5), -f_rz);
        }

        let v_world_linear = inputs.vel.linear;
        let v_local_angular = inputs.rot.transform_vector(&inputs.vel.angular);

        // TODO - pitch breaks this
        self.pid_vert_vel.update_setpoint(self.setpoints.ag_vel);
        self.outputs.ag_force.linear.y = self
            .pid_vert_vel
            .next_control_output(v_world_linear.y)
            .output;

        self.outputs.long_force.linear.x = self.setpoints.long_accel;

        self.pid_yaw_rate.update_setpoint(self.setpoints.yaw_rate);
        self.outputs.pseudo_torque.y = self
            .pid_yaw_rate
            .next_control_output(v_local_angular.y)
            .output;

        self.pid_pitch_rate
            .update_setpoint(self.setpoints.pitch_rate);
        self.outputs.pseudo_torque.z = self
            .pid_pitch_rate
            .next_control_output(v_local_angular.z)
            .output;

        &self.outputs
    }
}
