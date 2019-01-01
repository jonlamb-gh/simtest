// TODO
// - use margin on sizes/ect
// - move bits to impl, use half_extents()
// - make this a trait: pub fn update(&mut self, world: &World<f32>)
//
// use something like:
// https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/position_controller_pid.c
// for example controller
//
// use https://crates.io/crates/pid crate

use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::lag_engine::LAGEngine;
use crate::na;
use crate::na::{Isometry3, Point3, Vector3};
use crate::power_distribution::{Control, Engine, PowerDistribution};
use crate::velocity_controller::VelocityController;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::FreeJoint;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use std::collections::HashMap;

pub struct Platform {
    body: BodyHandle,
    box_node: BoxNode,
    power_dist: PowerDistribution,
    vel_control: VelocityController,
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

        // Q0:Q3, Q0 is bottom left, clockwise
        let e0 = LAGEngine::new(Vector3::new(-rx, 0.0, -rz), root_body, window, world);
        let e1 = LAGEngine::new(Vector3::new(rx, 0.0, -rz), root_body, window, world);
        let e2 = LAGEngine::new(Vector3::new(rx, 0.0, rz), root_body, window, world);
        let e3 = LAGEngine::new(Vector3::new(-rx, 0.0, rz), root_body, window, world);

        let mut engines = HashMap::new();
        engines.insert(Engine::E0, e0);
        engines.insert(Engine::E1, e1);
        engines.insert(Engine::E2, e2);
        engines.insert(Engine::E3, e3);

        Platform {
            body: root_body,
            box_node,
            power_dist: PowerDistribution::new(engines),
            vel_control: VelocityController::new(),
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

    pub fn velocity(&self, world: &World<f32>) -> Velocity<f32> {
        let mbody = world
            .multibody_link(self.body)
            .expect("Body is not in the world");

        mbody.velocity().clone()
    }

    pub fn control(&mut self, desired_vel: f32, world: &mut World<f32>) {
        // TODO
        let _pos = self.position(world);
        let vel = self.velocity(world);

        let thrust = self.vel_control.update(vel.linear.y, desired_vel);
        println!(
            "des {} - act {} - thrust {}",
            desired_vel, vel.linear.y, thrust
        );

        let control = Control {
            roll_comp: 0.0,
            pitch_comp: 0.0,
            e0: thrust,
            e1: thrust,
            e2: thrust,
            e3: thrust,
        };

        self.power_dist.control_thrust(&control, world);
    }
}
