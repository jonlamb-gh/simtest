use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::na;
use crate::na::{Isometry3, Point3, Vector3};
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::FixedJoint;
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

pub struct LAGEngine {
    box_node: BoxNode,
    force: Force<f32>,
}

impl LAGEngine {
    pub fn new(
        pos: Vector3<f32>,
        parent: BodyHandle,
        window: &mut Window,
        world: &mut World<f32>,
    ) -> Self {
        let engine_size = Vector3::new(0.1, 0.4, 0.1);

        let color = Point3::new(1.0, 0.0, 0.0);
        let pos = Isometry3::new(pos, na::zero());
        let delta = na::one();

        let cuboid = Cuboid::new(engine_size);
        let geom = ShapeHandle::new(cuboid.clone());

        let rx = cuboid.half_extents().x;
        let ry = cuboid.half_extents().y;
        let rz = cuboid.half_extents().z;

        let body = world.add_multibody_link(
            parent,
            FixedJoint::new(pos),
            na::zero(),
            na::zero(),
            geom.inertia(1.0),
            geom.center_of_mass(),
        );

        let collision_handle = world.add_collider(
            COLLIDER_MARGIN,
            geom,
            body,
            Isometry3::identity(),
            Material::default(),
        );

        let box_node = BoxNode::new(collision_handle, world, delta, rx, ry, rz, color, window);

        LAGEngine {
            box_node,
            force: Force::zero(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.box_node.update(world);
    }
}
