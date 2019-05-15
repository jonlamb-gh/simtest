use crate::na::Point3;
use crate::part::PartDesc;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::Cuboid;
use nphysics3d::object::{Collider, ColliderHandle};
use nphysics3d::world::World;

pub fn build_scene_node(
    part: &PartDesc,
    collider: &Collider<f32>,
    color: Point3<f32>,
    window: &mut Window,
) -> SceneNode {
    let shape = Cuboid::new(part.size / 2.0);

    let margin = collider.margin();
    let rx = shape.half_extents().x + margin;
    let ry = shape.half_extents().y + margin;
    let rz = shape.half_extents().z + margin;

    let gx = rx * 2.0;
    let gy = ry * 2.0;
    let gz = rz * 2.0;

    let mut node = window.add_cube(gx, gy, gz);

    node.set_color(color.x, color.y, color.z);

    node.set_local_transformation(*collider.position());

    node
}

pub fn update_scene_node(collider: ColliderHandle, world: &World<f32>, node: &mut SceneNode) {
    let t = world.collider(collider).unwrap().position();
    node.set_local_transformation(*t);
}
