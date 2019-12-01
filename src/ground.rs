use crate::box_node::{build_scene_node, update_scene_node};
use crate::na::{Isometry3, Point3, Vector3};
use crate::part::{Part, PartDesc};
use kiss3d::scene::SceneNode;
use kiss3d::window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyPartHandle, ColliderDesc, ColliderHandle};
use nphysics3d::world::World;

pub struct Ground {
    body: BodyPartHandle,
    collider: ColliderHandle,
    node: SceneNode,
}

impl Part for Ground {
    fn part_desc() -> PartDesc {
        PartDesc {
            size: Vector3::new(200.0, 1.0, 200.0),
            mass: 1.0,
            density: 1.0,
        }
    }

    fn collider_desc() -> ColliderDesc<f32> {
        let part = Self::part_desc();
        let shape = Cuboid::new(part.size / 2.0);
        ColliderDesc::new(ShapeHandle::new(shape))
    }

    fn body_part(&self) -> BodyPartHandle {
        self.body
    }

    fn object(&self) -> ColliderHandle {
        self.collider
    }

    fn position(&self, world: &World<f32>) -> Isometry3<f32> {
        *world.collider(self.collider).unwrap().position()
    }

    fn velocity(&self, world: &World<f32>) -> Velocity<f32> {
        let body = world.collider(self.collider).unwrap().body();
        world.body(body).unwrap().part(0).unwrap().velocity()
    }
}

impl Ground {
    pub fn new(world: &mut World<f32>, window: &mut window::Window) -> Self {
        let part = Self::part_desc();
        let collider = Self::collider_desc()
            .translation(Vector3::y() * -part.size.y / 2.0)
            .build(world);
        let handle = collider.handle();
        let body = collider.body_part(0);

        let color = Point3::new(1.0, 1.0, 1.0);
        let mut node = build_scene_node(&part, collider, color, window);
        node.set_texture_from_memory(include_bytes!("../res/checkerboard.png"), "checkerboard");

        Ground {
            body,
            collider: handle,
            node,
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        update_scene_node(self.collider, world, &mut self.node);
    }
}
