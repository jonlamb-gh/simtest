// TODO
// - use margin on sizes/ect
// - move bits to impl, use half_extents()

use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::lag_engine::LAGEngine;
use crate::na;
use crate::na::{Isometry3, Point3, Vector3};
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::FreeJoint;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

pub struct Platform {
    box_node: BoxNode,
    e0: LAGEngine,
    e1: LAGEngine,
    e2: LAGEngine,
    e3: LAGEngine,
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

        let e0 = LAGEngine::new(Vector3::new(rx, 0.0, rz), root_body, window, world);
        let e1 = LAGEngine::new(Vector3::new(-rx, 0.0, rz), root_body, window, world);
        let e2 = LAGEngine::new(Vector3::new(rx, 0.0, -rz), root_body, window, world);
        let e3 = LAGEngine::new(Vector3::new(-rx, 0.0, -rz), root_body, window, world);

        Platform { box_node, e0, e1, e2, e3 }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.box_node.update(world);
        self.e0.update(world);
    }
}
