use crate::box_node::{build_scene_node, update_scene_node};
use crate::colors::Colors;
use crate::na::{Point3, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet,
    RigidBodyDesc,
};

pub struct Env {
    boxes: Vec<Box>,
}

impl Env {
    pub fn new(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        window: &mut Window,
    ) -> Self {
        // TODO - configs
        let margin = ColliderDesc::<f32>::default_margin();
        let start_y = 2.0 * margin;
        let density = 1.0;
        let mass = 1.0;

        let mut boxes = vec![];
        let small_rad = 0.5;
        let small_half_size = Vector3::repeat(small_rad);
        let small_cuboid = ShapeHandle::new(Cuboid::new(small_half_size));

        for x in &[-35, -25, -15, -5, 15, 25, 35] {
            for y in &[-35, -25, -15, -5, 15, 25, 35] {
                for h in 0..5 {
                    let delta_y = 2.0 * (small_rad + margin) + 0.3;
                    let body = RigidBodyDesc::new()
                        .translation(Vector3::new(
                            *x as _,
                            start_y + (h as f32 * delta_y),
                            *y as _,
                        ))
                        .mass(mass)
                        .build();
                    let handle = bodies.insert(body);
                    let co = ColliderDesc::new(small_cuboid.clone())
                        .density(density)
                        .build(BodyPartHandle(handle, 0));
                    let collider = colliders.insert(co);
                    let half_extents = small_half_size + Vector3::repeat(margin);

                    let color = Colors::new();
                    let node = build_scene_node(collider, colliders, half_extents, color, window);
                    boxes.push(Box::new(collider, node));
                }
            }
        }

        Env { boxes }
    }

    // TODO
    // pub fn reset()

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        for b in &mut self.boxes {
            b.update(colliders);
        }
    }
}

struct Box {
    collider: DefaultColliderHandle,
    node: SceneNode,
    // TODO - reset_position
}

impl Box {
    pub fn new(collider: DefaultColliderHandle, node: SceneNode) -> Self {
        Box { collider, node }
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        update_scene_node(self.collider, colliders, &mut self.node);
    }
}
