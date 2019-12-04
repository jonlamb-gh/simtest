use crate::box_node::update_scene_node;
use crate::na::Vector3;
use kiss3d::scene::SceneNode;
use nphysics3d::object::{DefaultColliderHandle, DefaultColliderSet};

pub struct BaseFrame {
    collider: DefaultColliderHandle,
    node: SceneNode,
}

impl BaseFrame {
    pub fn new(collider: DefaultColliderHandle, node: SceneNode) -> Self {
        BaseFrame { collider, node }
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        update_scene_node(self.collider, colliders, &mut self.node);
    }

    pub fn size() -> Vector3<f32> {
        Vector3::new(2.0, 0.10, 1.0)
    }
}
