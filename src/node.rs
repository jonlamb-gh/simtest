//! Mostly a copy of nphysics/nphysics_testbed3d/src/objects/node.rs

// TODO
#![allow(dead_code)]

use crate::box_node::BoxNode;
use crate::na::{Isometry3, Point3};
use kiss3d::scene::SceneNode;
use nphysics3d::object::ColliderHandle;
use nphysics3d::world::World;

pub enum Node {
    Box(BoxNode),
}

impl Node {
    pub fn select(&mut self) {
        match *self {
            Node::Box(ref mut n) => n.select(),
        }
    }

    pub fn unselect(&mut self) {
        match *self {
            Node::Box(ref mut n) => n.unselect(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        match *self {
            Node::Box(ref mut n) => n.update(world),
        }
    }

    pub fn scene_node(&self) -> &SceneNode {
        match *self {
            Node::Box(ref n) => n.scene_node(),
        }
    }

    pub fn scene_node_mut(&mut self) -> &mut SceneNode {
        match *self {
            Node::Box(ref mut n) => n.scene_node_mut(),
        }
    }

    pub fn collider(&self) -> ColliderHandle {
        match *self {
            Node::Box(ref n) => n.object(),
        }
    }

    pub fn set_color(&mut self, color: Point3<f32>) {
        match *self {
            Node::Box(ref mut n) => n.set_color(color),
        }
    }
}

pub fn update_scene_node(
    node: &mut SceneNode,
    world: &World<f32>,
    coll: ColliderHandle,
    color: &Point3<f32>,
    delta: &Isometry3<f32>,
) {
    let co = world.collider(coll).unwrap();
    // let active = world.body(co.data().body()).is_active();

    if true {
        // active {
        node.set_local_transformation(co.position() * delta);
        node.set_color(color.x, color.y, color.z);
    } else {
        node.set_color(color.x * 0.25, color.y * 0.25, color.z * 0.25);
    }
}
