// TODO
// - use margin on sizes/ect
// - move bits to impl, use half_extents()
// - make this a trait: pub fn update(&mut self, world: &World<f32>)
//
// use something like:
// https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/position_controller_pid.c
// for example controller
//
// linear_at_point() and rigid body instead of joints?

use crate::attitude_controller::AttitudeController;
use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::lag_engine::LAGEngine;
use crate::na;
use crate::na::geometry::UnitQuaternion;
use crate::na::{Isometry3, Point3, Vector3};
use crate::power_distribution::{Control, PowerDistribution};
use crate::velocity_controller::VelocityController;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::FreeJoint;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

pub struct Platform {
    body: BodyHandle,
    box_node: BoxNode,
    power_dist: PowerDistribution,
    power_dist_control: Control,
    vel_control: VelocityController,
    vel_setpoint: Velocity<f32>,
    att_control: AttitudeController,
    att_setpoint: UnitQuaternion<f32>,
}

impl Platform {
    pub fn new(pos: Vector3<f32>, window: &mut Window, world: &mut World<f32>) -> Self {
        let platform_size = Vector3::new(3.0, 0.2, 1.8);

        let color = Point3::new(1.0, 0.0, 0.0);
        let pos = Isometry3::new(pos, na::zero());
        let delta = na::one();

        let cuboid = Cuboid::new(platform_size);
        let geom = ShapeHandle::new(cuboid.clone());

        let root_body = world.add_multibody_link(
            BodyHandle::ground(),
            FreeJoint::new(pos),
            na::zero(),
            na::zero(),
            geom.inertia(1.0),
            geom.center_of_mass(),
        );

        let collision_handle = world.add_collider(
            COLLIDER_MARGIN,
            geom,
            root_body,
            Isometry3::identity(),
            Material::default(),
        );

        let rx = cuboid.half_extents().x;
        let ry = cuboid.half_extents().y;
        let rz = cuboid.half_extents().z;

        let box_node = BoxNode::new(collision_handle, world, delta, rx, ry, rz, color, window);

        let engine_size = Vector3::new(0.1, 0.4, 0.1);
        let e2 = LAGEngine::new(
            Vector3::new(-rx, 0.0, -rz),
            engine_size,
            root_body,
            window,
            world,
        );
        let e3 = LAGEngine::new(
            Vector3::new(rx, 0.0, -rz),
            engine_size,
            root_body,
            window,
            world,
        );
        let e0 = LAGEngine::new(
            Vector3::new(rx, 0.0, rz),
            engine_size,
            root_body,
            window,
            world,
        );
        let e1 = LAGEngine::new(
            Vector3::new(-rx, 0.0, rz),
            engine_size,
            root_body,
            window,
            world,
        );

        Platform {
            body: root_body,
            box_node,
            power_dist: PowerDistribution::new(e0, e1, e2, e3),
            power_dist_control: Control::new(),
            vel_control: VelocityController::new(),
            vel_setpoint: Velocity::zero(),
            att_control: AttitudeController::new(),
            att_setpoint: UnitQuaternion::identity(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.box_node.update(world);
        self.power_dist.update(world);
    }

    pub fn reset(&mut self, world: &mut World<f32>) {
        // TODO - doesn't work yet
        let mbody = world
            .multibody_mut(self.body)
            .expect("Body is not in the world");
        mbody.clear_dynamics();
        self.power_dist.reset(world);
    }

    pub fn position(&self, world: &World<f32>) -> Isometry3<f32> {
        let mbody = world
            .multibody_link(self.body)
            .expect("Body is not in the world");

        mbody.position()
    }

    /// Global frame
    pub fn velocity(&self, world: &World<f32>) -> Velocity<f32> {
        let mbody = world
            .multibody_link(self.body)
            .expect("Body is not in the world");

        mbody.velocity().clone()
    }

    /// Platform frame
    pub fn rel_velocity(&self, world: &World<f32>) -> Velocity<f32> {
        // TODO - fix this
        let iso = self.position(world);
        self.velocity(world).rotated(&iso.rotation)
    }

    pub fn velocity_setpoint(&self) -> &Velocity<f32> {
        &self.vel_setpoint
    }

    pub fn attitude_setpoint(&self) -> &UnitQuaternion<f32> {
        &self.att_setpoint
    }

    pub fn power_dist_control(&self) -> &Control {
        &self.power_dist_control
    }

    pub fn step_controls(
        &mut self,
        vel_setpoint: Velocity<f32>,
        att_setpoint: UnitQuaternion<f32>,
        world: &mut World<f32>,
    ) {
        self.vel_setpoint = vel_setpoint;
        self.att_setpoint = att_setpoint;

        let pos = self.position(world);
        let rot = pos.rotation;
        let vel = self.velocity(world);

        let thrust = self.vel_control.update(vel, self.vel_setpoint);
        let torque = self.att_control.update(rot, self.att_setpoint);

        self.power_dist_control.e0 = Force::linear(Vector3::new(0.0, 0.0, 0.0));

        /*
        let pos = self.position(world);
        let rot = pos.rotation;
        let mut vel = self.velocity(world);
        let rel_vel = self.rel_velocity(world);
        let dyaw = desired_vel.angular.y;
        let rot_v = rot.scaled_axis();
        let desired_rot = UnitQuaternion::new(Vector3::new(rot_v.x, rot_v.y + dyaw, rot_v.z));

        // Use relative for x/z
        vel.linear.x = rel_vel.linear.x;
        vel.linear.z = rel_vel.linear.z;

        let thrust = self.vel_control.update(vel, desired_vel);

        let torque = self.att_control.update(rot, desired_rot);

        let control = Control {
            roll_comp: 0.0,
            pitch_comp: 0.0,
            yaw_comp: 0.0,
            e0: thrust.linear.y,
            e1: thrust.linear.y,
            e2: thrust.linear.y,
            e3: thrust.linear.y,
            e4: thrust.linear.x,
            e5: thrust.linear.z,
            ty: torque.angular.y,
        };

        self.power_dist.control_thrust(&control, world);
        */
    }
}
