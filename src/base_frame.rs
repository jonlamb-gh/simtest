use crate::box_node::update_scene_node;
use crate::na::Vector3;
use kiss3d::scene::SceneNode;
use nphysics3d::object::{
    BodyPartHandle, DefaultBodyHandle, DefaultColliderHandle, DefaultColliderSet,
};

pub struct BaseFrame {
    body_part: BodyPartHandle<DefaultBodyHandle>,
    collider: DefaultColliderHandle,
    node: SceneNode,
}

impl BaseFrame {
    pub fn new(
        body_part: BodyPartHandle<DefaultBodyHandle>,
        collider: DefaultColliderHandle,
        node: SceneNode,
    ) -> Self {
        BaseFrame {
            body_part,
            collider,
            node,
        }
    }

    pub fn body_part(&self) -> &BodyPartHandle<DefaultBodyHandle> {
        &self.body_part
    }

    pub fn collider(&self) -> &DefaultColliderHandle {
        &self.collider
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        update_scene_node(self.collider, colliders, &mut self.node);
    }

    pub fn size() -> Vector3<f32> {
        Vector3::new(2.0, 0.10, 1.0)
    }
}
