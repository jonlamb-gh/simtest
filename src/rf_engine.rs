use crate::box_node::update_scene_node;
use crate::force_gen::ForceGen;
use crate::na::{Isometry3, Point3, Rotation3, Vector3};
use crate::part::{Part, PartDesc};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::ForceGeneratorHandle;
use nphysics3d::joint::FixedJoint;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyHandle, ColliderDesc, ColliderHandle, MultibodyDesc};
use nphysics3d::world::World;

pub struct RfEngine {
    body: BodyHandle,
    collider: ColliderHandle,
    node: SceneNode,
    force_gen: ForceGeneratorHandle,
}

impl Part for RfEngine {
    fn part_desc() -> PartDesc {
        PartDesc {
            size: Vector3::new(0.1, 0.1, 0.1),
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

impl RfEngine {
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

    pub fn new(
        body: BodyHandle,
        collider: ColliderHandle,
        node: SceneNode,
        force_gen: ForceGeneratorHandle,
    ) -> Self {
        RfEngine {
            body,
            collider,
            node,
            force_gen,
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        update_scene_node(self.collider, world, &mut self.node);
    }

    // TODO - this is the world frame, make this relative, update force vector after
    pub fn force(&self, world: &World<f32>) -> Force<f32> {
        let force_gen = world
            .force_generator(self.force_gen)
            .downcast_ref::<ForceGen>()
            .unwrap();
        force_gen.force()
    }

    // TODO Apply a relative force
    pub fn set_force(&mut self, force: Force<f32>, world: &mut World<f32>) {
        let p = self.position(world);
        // TODO - issues with ForceGen local force/point misunderstandings
        // seems like the force is applied until body moves, like the point
        // is in the world frame?
        //
        //let rotation = Rotation3::from(p.rotation);
        //println!("t {:#?}", p.translation);
        //println!("{:#?}", p.rotation.euler_angles());
        //let force = Force::linear(p.rotation.inverse() * force.linear);
        //let force = Force::linear(rotation * force.linear);
        let force = Force::linear(p.rotation.transform_vector(&force.linear));
        //let force =
        // Force::linear(p.rotation.inverse().transform_vector(&force.linear));
        println!("f {:#?}", force.linear);

        // TODO
        // switch to a velocity model for now
        //  body.set_velocity(Velocity::linear(velocity_x, 0.0));
        //  https://github.com/rustsim/nphysics/issues/124

        let force_gen = world
            .force_generator_mut(self.force_gen)
            .downcast_mut::<ForceGen>()
            .unwrap();
        force_gen.set_force(force);
    }

    pub fn draw_force_vector(&self, world: &World<f32>, win: &mut Window) {
        // TODO - configs
        let color = Point3::new(0.0, 0.0, 1.0);
        let scale = 0.25;

        let a = self.position(world).translation.vector;
        let b = a + (self.force(world).linear * scale);

        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
    }
}
