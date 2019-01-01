use crate::config::THRUST_LIMIT;
use crate::na::Vector3;
use nphysics3d::math::{Force, Velocity};
use pid::Pid;

pub struct VelocityController {
    pid_vx: Pid<f32>,
    pid_vy: Pid<f32>,
    pid_vz: Pid<f32>,
    setpoint: Velocity<f32>,
}

impl VelocityController {
    pub fn new() -> Self {
        // TODO - PID configs
        let kp = 25.0;
        let ki = 10.0;
        let kd = 0.0;
        let p_limit = THRUST_LIMIT;
        let i_limit = 15.0;
        let d_limit = 10.0;
        let mut pid_vx = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_vx.update_setpoint(0.0);

        let kp = 25.0;
        let ki = 10.0;
        let kd = 0.0;
        let p_limit = THRUST_LIMIT;
        let i_limit = 15.0;
        let d_limit = 10.0;
        let mut pid_vy = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_vy.update_setpoint(0.0);

        let kp = 25.0;
        let ki = 10.0;
        let kd = 0.0;
        let p_limit = THRUST_LIMIT;
        let i_limit = 15.0;
        let d_limit = 10.0;
        let mut pid_vz = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_vz.update_setpoint(0.0);

        VelocityController {
            pid_vx,
            pid_vy,
            pid_vz,
            setpoint: Velocity::zero(),
        }
    }

    pub fn update(&mut self, input: Velocity<f32>, setpoint: Velocity<f32>) -> Force<f32> {
        self.setpoint = setpoint;

        self.pid_vx.update_setpoint(setpoint.linear.x);
        let out_x = self.pid_vx.next_control_output(input.linear.x).output;

        self.pid_vy.update_setpoint(setpoint.linear.y);
        let out_y = self.pid_vy.next_control_output(input.linear.y).output;

        self.pid_vz.update_setpoint(setpoint.linear.z);
        let out_z = self.pid_vz.next_control_output(input.linear.z).output;

        Force::linear(Vector3::new(out_x, out_y, out_z))
    }

    pub fn get_setpoint(&self) -> Velocity<f32> {
        self.setpoint
    }
}
