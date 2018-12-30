use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::na;
use crate::na::{Isometry3, Point3, Vector3};
use crate::node;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FixedJoint, FreeJoint, Joint};
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

pub struct LAGEngine {
    box_node: BoxNode,
    force: Force<f32>,
}

impl LAGEngine {
    pub fn new(node: BoxNode) -> Self {
        LAGEngine {
            box_node: node,
            force: Force::zero(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.box_node.update(world);
    }

    pub fn add_to_body(
        pos: Vector3<f32>,
        world: &mut World<f32>,
        shape: ShapeHandle<f32>,
        parent: BodyHandle,
    ) -> BodyHandle {
        let handle = world.add_multibody_link(
            parent,
            FixedJoint::new(Isometry3::new(pos, na::zero())),
            na::zero(),
            na::zero(),
            shape.inertia(1.0),
            shape.center_of_mass(),
        );

        /*
        world.add_collider(
            COLLIDER_MARGIN,
            shape,
            handle,
            Isometry3::identity(),
            Material::default(),
        );
        */

        handle
    }

    pub fn shape(size: Vector3<f32>) -> ShapeHandle<f32> {
        let cuboid = Cuboid::new(size);
        ShapeHandle::new(cuboid)
    }
}
