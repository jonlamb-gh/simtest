use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::na;
use crate::na::{Isometry3, Point3, Vector3};
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::{ForceGenerator, ForceGeneratorHandle};
use nphysics3d::joint::FixedJoint;
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, BodySet, Material};
use nphysics3d::solver::IntegrationParameters;
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

pub struct LAGEngine {
    force_gen: ForceGeneratorHandle,
    node: BoxNode,
}

struct ForceGen {
    body: BodyHandle,
    force: Force<f32>,
}

impl LAGEngine {
    pub fn new(
        pos: Vector3<f32>,
        size: Vector3<f32>,
        parent: BodyHandle,
        window: &mut Window,
        world: &mut World<f32>,
    ) -> Self {
        let engine_size = size;

        let color = Point3::new(1.0, 0.0, 0.0);
        let delta = na::one();

        let cuboid = Cuboid::new(engine_size);
        let geom = ShapeHandle::new(cuboid.clone());

        let rx = cuboid.half_extents().x;
        let ry = cuboid.half_extents().y;
        let rz = cuboid.half_extents().z;

        let body = world.add_multibody_link(
            parent,
            FixedJoint::new(Isometry3::new(pos, na::zero())),
            na::zero(),
            na::zero(),
            geom.inertia(1.0),
            geom.center_of_mass(),
        );

        let collision_handle = world.add_collider(
            COLLIDER_MARGIN,
            geom,
            body,
            Isometry3::identity(),
            Material::default(),
        );

        let node = BoxNode::new(collision_handle, world, delta, rx, ry, rz, color, window);

        let force_gen = ForceGen {
            body,
            force: Force::zero(),
        };

        let force_gen_handle = world.add_force_generator(force_gen);

        LAGEngine {
            node,
            force_gen: force_gen_handle,
        }
    }

    pub fn set_force(&mut self, force: Force<f32>, world: &mut World<f32>) {
        let force_gen = world
            .force_generator_mut(self.force_gen)
            .downcast_mut::<ForceGen>()
            .unwrap();
        force_gen.force = force;
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.node.update(world);
    }
}

impl ForceGenerator<f32> for ForceGen {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        if bodies.contains(self.body) {
            let mut part = bodies.body_part_mut(self.body);
            part.apply_force(&self.force);
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}
