use crate::config::TORQUE_LIMIT;
use crate::na::geometry::UnitQuaternion;
use crate::na::Vector3;
use nphysics3d::math::Force;
use pid::Pid;

pub struct AttitudeController {
    pid_yaw: Pid<f32>,
    setpoint: UnitQuaternion<f32>,
}

impl AttitudeController {
    pub fn new() -> Self {
        // TODO - PID configs
        let kp = 15.0;
        let ki = 5.0;
        let kd = 0.0;
        let p_limit = TORQUE_LIMIT;
        let i_limit = 15.0;
        let d_limit = 10.0;
        let mut pid_yaw = Pid::new(kp, ki, kd, p_limit, i_limit, d_limit);
        pid_yaw.update_setpoint(0.0);

        AttitudeController {
            pid_yaw,
            setpoint: UnitQuaternion::identity(),
        }
    }

    pub fn update(
        &mut self,
        input: UnitQuaternion<f32>,
        setpoint: UnitQuaternion<f32>,
    ) -> Force<f32> {
        self.setpoint = setpoint;

        self.pid_yaw.update_setpoint(setpoint.euler_angles().2);
        let out_y = self
            .pid_yaw
            .next_control_output(input.euler_angles().2)
            .output;

        Force::torque(Vector3::new(0.0, out_y, 0.0))
    }

    pub fn get_setpoint(&self) -> UnitQuaternion<f32> {
        self.setpoint
    }
}