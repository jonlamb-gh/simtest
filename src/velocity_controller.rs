use crate::config::THRUST_LIMIT;
use pid::Pid;
//use nphysics3d::math::Velocity;

pub struct VelocityController {
    pid_vy: Pid<f32>,
}

impl VelocityController {
    pub fn new() -> Self {
        // TODO - PID configs
        let kp = 25.0;
        let ki = 15.0;
        let kd = 0.1;
        let p_limit = THRUST_LIMIT;
        let i_limit = 15.0;
        let d_limit = 10.0;
        let mut pid_vy = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_vy.update_setpoint(0.0);

        VelocityController { pid_vy }
    }

    //pub fn update(&mut self, input: Velocity<f32>, setpoint: Velocity<f32>) ->
    // f32 {
    pub fn update(&mut self, input: f32, setpoint: f32) -> f32 {
        self.pid_vy.update_setpoint(setpoint);
        self.pid_vy.next_control_output(input).output
    }
}
