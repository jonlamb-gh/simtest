use crate::box_node::{build_scene_node, update_scene_node};
use crate::na::{Point3, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet,
    Ground as GroundObj,
};

pub struct Ground {
    collider: DefaultColliderHandle,
    node: SceneNode,
}

impl Ground {
    pub fn new(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        window: &mut window::Window,
    ) -> Self {
        let half_size = Self::size() / 2.0;
        let ground_shape = ShapeHandle::new(Cuboid::new(half_size));
        let ground_handle = bodies.insert(GroundObj::new());

        let co = ColliderDesc::new(ground_shape)
            .translation(Vector3::y() * -Self::size().y)
            .build(BodyPartHandle(ground_handle, 0));
        let margin = co.margin();
        let half_extents = half_size + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(1.0, 1.0, 1.0);
        let mut node = build_scene_node(collider, colliders, half_extents, color, window);
        node.set_texture_from_memory(include_bytes!("../res/checkerboard.png"), "checkerboard");

        Ground { collider, node }
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        update_scene_node(self.collider, colliders, &mut self.node);
    }

    pub fn size() -> Vector3<f32> {
        Vector3::new(200.0, 0.2, 200.0)
    }
}
