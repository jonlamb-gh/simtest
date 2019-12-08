// TODO - move this stuff to a lib crate for easier testing

// https://crates.io/crates/pid
// https://docs.rs/pid/2.0.0/pid/index.html

use crate::na::Vector3;
use nphysics3d::math::{Force, Isometry, Velocity};
use pid::Pid;

#[derive(Debug)]
pub struct SetPoints {
    pub angular_velocity: Vector3<f32>,
    pub linear_velocity: Vector3<f32>,
}

#[derive(Debug)]
pub struct Sensors {
    // NOTE: only need the .rotation UnitQuaternion
    /// Position and orientation/rotation of the platform's RigidBody
    pub iso: Isometry<f32>,

    /// Linear and angular velocity of the platform's RigidBody
    pub vel: Velocity<f32>,
}

/// Forces relative to the platform's frame
#[derive(Debug)]
pub struct Outputs {
    pub rfe_fl_force: Force<f32>,
    pub rfe_fr_force: Force<f32>,
    pub rfe_rl_force: Force<f32>,
    pub rfe_rr_force: Force<f32>,
}

pub struct Controller {
    front_thrust_proportion: f32,
    pid_rx: Pid<f32>,
    pid_ry: Pid<f32>,
    pid_rz: Pid<f32>,
    pid_vx: Pid<f32>,
    pid_vy: Pid<f32>,
    pid_vz: Pid<f32>,
    outputs: Outputs,
}

const KP_X: f32 = 1.0;
const KI_X: f32 = 0.0;
const KD_X: f32 = 0.0;
const LIMIT_X: f32 = 1.0;

const KP_Y: f32 = 1.0;
const KI_Y: f32 = 0.0;
const KD_Y: f32 = 0.0;
const LIMIT_Y: f32 = 1.0;

const KP_Z: f32 = 1.0;
const KI_Z: f32 = 0.0;
const KD_Z: f32 = 0.0;
const LIMIT_Z: f32 = 1.0;

const KP_V: f32 = 60.0;
const KI_V: f32 = 0.0;
const KD_V: f32 = 0.0;
const LIMIT_V: f32 = 200.0;

impl Controller {
    pub fn new() -> Self {
        Controller {
            front_thrust_proportion: 0.5,
            pid_rx: Pid::new(KP_X, KI_X, KD_X, LIMIT_X, LIMIT_X, LIMIT_X, 0.0),
            pid_ry: Pid::new(KP_Y, KI_Y, KD_Y, LIMIT_Y, LIMIT_Y, LIMIT_Y, 0.0),
            pid_rz: Pid::new(KP_Z, KI_Z, KD_Z, LIMIT_Z, LIMIT_Z, LIMIT_Z, 0.0),
            pid_vx: Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0),
            pid_vy: Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0),
            pid_vz: Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0),
            outputs: Outputs {
                rfe_fl_force: Force::zero(),
                rfe_fr_force: Force::zero(),
                rfe_rl_force: Force::zero(),
                rfe_rr_force: Force::zero(),
            },
        }
    }

    pub fn reset(&mut self) {
        self.pid_rx = Pid::new(KP_X, KI_X, KD_X, LIMIT_X, LIMIT_X, LIMIT_X, 0.0);
        self.pid_ry = Pid::new(KP_Y, KI_Y, KD_Y, LIMIT_Y, LIMIT_Y, LIMIT_Y, 0.0);
        self.pid_rz = Pid::new(KP_Z, KI_Z, KD_Z, LIMIT_Z, LIMIT_Z, LIMIT_Z, 0.0);
        self.pid_vx = Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0);
        self.pid_vy = Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0);
        self.pid_vz = Pid::new(KP_V, KI_V, KD_V, LIMIT_V, LIMIT_V, LIMIT_V, 0.0);
        self.outputs.rfe_fl_force = Force::zero();
        self.outputs.rfe_fr_force = Force::zero();
        self.outputs.rfe_rl_force = Force::zero();
        self.outputs.rfe_rr_force = Force::zero();
    }

    pub fn update(&mut self, set_points: &SetPoints, sensors: &Sensors) -> &Outputs {
        self.outputs.rfe_fl_force = Force::zero();
        self.outputs.rfe_fr_force = Force::zero();
        self.outputs.rfe_rl_force = Force::zero();
        self.outputs.rfe_rr_force = Force::zero();

        self.distribute_velocity_control_thrust(set_points, sensors);

        //self.distribute_longitudinal_thrust(set_points);
        //self.distribute_vertical_thrust(set_points);

        self.distribute_rotational_thrust(set_points, sensors);

        &self.outputs
    }

    fn distribute_velocity_control_thrust(&mut self, set_points: &SetPoints, sensors: &Sensors) {
        let aligned_linear_velocity = sensors
            .iso
            .rotation
            .inverse()
            .transform_vector(&sensors.vel.linear);

        self.pid_vx.setpoint = set_points.linear_velocity.x;
        let vx_output = self
            .pid_vx
            .next_control_output(aligned_linear_velocity.x)
            .output;
        //println!(
        //    "sp {} -- in {} -- out {}",
        //    set_points.linear_velocity.x, aligned_linear_velocity.x, vx_output
        //);

        self.pid_vy.setpoint = set_points.linear_velocity.y;
        let vy_output = self
            .pid_vy
            .next_control_output(aligned_linear_velocity.y)
            .output;

        self.pid_vz.setpoint = set_points.linear_velocity.z;
        let vz_output = self
            .pid_vz
            .next_control_output(aligned_linear_velocity.z)
            .output;

        let thrust = Vector3::new(vx_output, vy_output, vz_output);
        let quarter_thrust = thrust / 4.0;

        // TODO front_thrust_proportion
        self.outputs.rfe_fl_force.linear += quarter_thrust;
        self.outputs.rfe_fr_force.linear += quarter_thrust;
        self.outputs.rfe_rl_force.linear += quarter_thrust;
        self.outputs.rfe_rr_force.linear += quarter_thrust;
    }

    fn distribute_rotational_thrust(&mut self, set_points: &SetPoints, sensors: &Sensors) {
        let aligned_angular_velocity = sensors
            .iso
            .rotation
            .inverse()
            .transform_vector(&sensors.vel.angular);

        // Rx, roll rate
        self.pid_rx.setpoint = set_points.angular_velocity.x;
        let rx_output = self
            .pid_rx
            .next_control_output(aligned_angular_velocity.x)
            .output;
        self.outputs.rfe_fl_force.linear.y += rx_output / 2.0;
        self.outputs.rfe_fr_force.linear.y += -rx_output / 2.0;
        //println!(
        //    "sp {} -- in {} -- out {}",
        //    set_points.angular_velocity.x, sensors.vel.angular.x, rx_output
        //);

        // Ry, yaw rate
        self.pid_ry.setpoint = set_points.angular_velocity.y;
        let ry_output = self
            .pid_ry
            .next_control_output(aligned_angular_velocity.y)
            .output;
        self.outputs.rfe_fl_force.linear.z += -ry_output / 2.0;
        self.outputs.rfe_fr_force.linear.z += -ry_output / 2.0;

        // Rz, pitch rate
        self.pid_rz.setpoint = set_points.angular_velocity.z;
        let rz_output = self
            .pid_rz
            .next_control_output(aligned_angular_velocity.z)
            .output;
        self.outputs.rfe_fl_force.linear.y += rz_output / 2.0;
        self.outputs.rfe_fr_force.linear.y += rz_output / 2.0;
    }
}
