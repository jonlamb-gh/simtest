use crate::box_node::update_scene_node;
use crate::na::{Isometry3, Point3, Vector3};
use crate::part::{Part, PartDesc};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::algebra::ForceType;
use nphysics3d::joint::FixedJoint;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyHandle, ColliderDesc, ColliderHandle, MultibodyDesc};
use nphysics3d::world::World;

pub struct AgEngine {
    body: BodyHandle,
    collider: ColliderHandle,
    node: SceneNode,
    force: Force<f32>,
}

impl Part for AgEngine {
    fn part_desc() -> PartDesc {
        PartDesc {
            size: Vector3::new(0.08, 0.08, 0.08),
            mass: 0.1,
            density: 0.3,
        }
    }

    fn collider_desc() -> ColliderDesc<f32> {
        let part = Self::part_desc();
        let shape = Cuboid::new(part.size / 2.0);
        ColliderDesc::new(ShapeHandle::new(shape)).density(part.density)
    }

    fn body(&self) -> BodyHandle {
        self.body
    }

    fn object(&self) -> ColliderHandle {
        self.collider
    }

    fn position(&self, world: &World<f32>) -> Isometry3<f32> {
        *world.collider(self.collider).unwrap().position()

        // TODO - is the part or the parent?
        //world.body(self.body).unwrap().part(0).unwrap().position()
    }

    fn velocity(&self, world: &World<f32>) -> Velocity<f32> {
        // TODO - is the part or the parent?
        //world.body(self.body).unwrap().part(0).unwrap().velocity()

        let body = world.collider(self.collider).unwrap().body();
        world.body(body).unwrap().part(0).unwrap().velocity()
    }
}

impl AgEngine {
    pub fn build_link<'a>(
        name: String,
        translation: Vector3<f32>,
        collider: &'a ColliderDesc<f32>,
        mut mbody: MultibodyDesc<'a, f32>,
    ) -> MultibodyDesc<'a, f32> {
        let part = Self::part_desc();

        //let joint = FixedJoint::new(Isometry3::new(translation, na::zero()));
        let joint = FixedJoint::new(Isometry3::identity());

        mbody
            .add_child(joint)
            .add_collider(collider)
            .set_mass(part.mass)
            .set_parent_shift(translation)
            .set_name(name);

        mbody
    }

    pub fn new(body: BodyHandle, collider: ColliderHandle, node: SceneNode) -> Self {
        AgEngine {
            body,
            collider,
            node,
            force: Force::zero(),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        update_scene_node(self.collider, world, &mut self.node);
    }

    pub fn force(&self) -> Force<f32> {
        self.force
    }

    pub fn set_force(&mut self, force: Force<f32>, world: &mut World<f32>) {
        //        let ag_force = Force::linear(Vector3::new(0.0, force.linear.y, 0.0));
        //        world
        //            .force_generator_mut(self.force_gen)
        //            .downcast_mut::<ForceGen>()
        //            .unwrap()
        //            .set_force(ag_force);

        self.force = Force::linear(Vector3::new(0.0, force.linear.y, 0.0));

        let body = world.body_mut(self.body).unwrap();
        body.apply_force(0, &self.force, ForceType::Force, false);

        //let body = world.collider(self.collider).unwrap().body();
        //world
        //    .body_mut(body)
        //    .unwrap()
        //    .apply_force(0, &self.force, ForceType::Force, true);
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
