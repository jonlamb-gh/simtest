use crate::na::{Point3, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nphysics3d::object::{DefaultColliderHandle, DefaultColliderSet};

pub fn build_scene_node(
    collider: DefaultColliderHandle,
    colliders: &DefaultColliderSet<f32>,
    half_extents: Vector3<f32>,
    color: Point3<f32>,
    window: &mut Window,
) -> SceneNode {
    let extents = half_extents * 2.0;
    let mut node = window.add_cube(extents.x, extents.y, extents.z);
    node.set_color(color.x, color.y, color.z);
    node.set_local_transformation(*colliders.get(collider).unwrap().position());
    node
}

pub fn update_scene_node(
    collider: DefaultColliderHandle,
    colliders: &DefaultColliderSet<f32>,
    node: &mut SceneNode,
) {
    node.set_local_transformation(*colliders.get(collider).unwrap().position());
}
