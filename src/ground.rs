use crate::na::{self, Point3, Vector3};
use kiss3d::window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::ColliderDesc;
use nphysics3d::world::World;
use nphysics_testbed3d::objects::box_node;

pub struct Ground {
    node: box_node::Box,
}

impl Ground {
    pub fn new(world: &mut World<f32>, window: &mut window::Window) -> Self {
        let delta = na::one();
        let size = Vector3::new(100.0, 1.0, 100.0);
        let shape = Cuboid::new(size / 2.0);
        let color = Point3::new(0.4, 0.4, 0.4);

        let collider = ColliderDesc::new(ShapeHandle::new(shape.clone()))
            .translation(Vector3::y() * -size.y / 2.0)
            .build(world);

        let margin = collider.margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;
        let rz = shape.half_extents().z + margin;

        Ground {
            node: box_node::Box::new(collider.handle(), world, delta, rx, ry, rz, color, window),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.node.update(world);
    }
}
