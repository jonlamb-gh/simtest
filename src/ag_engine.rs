use crate::box_node::{build_scene_node, update_scene_node};
use crate::na::{Isometry3, Point3, Vector3};
use crate::part::{Part, PartDesc};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use ncollide3d::world::CollisionGroups;
use nphysics3d::algebra::ForceType;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyPartHandle, ColliderDesc, ColliderHandle};
use nphysics3d::world::World;

pub struct AgEngine {
    body: BodyPartHandle,
    collider: ColliderHandle,
    node: SceneNode,
    force: Force<f32>,
}

impl Part for AgEngine {
    fn part_desc() -> PartDesc {
        PartDesc {
            size: Vector3::new(0.08, 0.08, 0.08),
            mass: 0.1,
            density: 1.0,
        }
    }

    fn collider_desc() -> ColliderDesc<f32> {
        let part = Self::part_desc();
        let shape = Cuboid::new(part.size / 2.0);
        ColliderDesc::new(ShapeHandle::new(shape)).density(part.density)
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

impl AgEngine {
    pub fn new(
        parent: BodyPartHandle,
        translation: Vector3<f32>,
        world: &mut World<f32>,
        window: &mut Window,
    ) -> Self {
        // TODO - configs
        let mut group = CollisionGroups::new();
        group.set_membership(&[1]);
        group.set_whitelist(&[1]);

        let part = Self::part_desc();
        let collider_desc = Self::collider_desc()
            .translation(translation)
            .collision_groups(group);
        let collider = collider_desc.build_with_parent(parent, world).unwrap();
        let handle = collider.handle();

        let color = Point3::new(0.0, 0.6352, 1.0);
        let node = build_scene_node(&part, collider, color, window);

        AgEngine {
            body: parent,
            collider: handle,
            node,
            force: Force::zero(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        update_scene_node(self.collider, world, &mut self.node);
    }

    pub fn force(&self) -> &Force<f32> {
        &self.force
    }

    // World frame
    pub fn set_force(&mut self, force: Force<f32>, world: &mut World<f32>) {
        self.force = Force::linear(Vector3::new(0.0, force.linear.y, 0.0));
        let body = world.body_mut(self.body.0).unwrap();
        body.apply_force(0, &self.force, ForceType::Force, true);
    }

    pub fn draw_force_vector(&self, world: &World<f32>, win: &mut Window) {
        // TODO - configs
        let color = Point3::new(0.0, 0.0, 1.0);
        let scale = 0.1;

        let a = self.position(world).translation.vector;
        let b = a + (self.force().linear * scale);

        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
    }
}
