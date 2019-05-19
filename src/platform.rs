use crate::ag_engine::AgEngine;
use crate::box_node::{build_scene_node, update_scene_node};
use crate::controller::ControlSetpoints;
use crate::na::{Isometry3, Matrix3, Point3, Vector3};
use crate::part::{Part, PartDesc};
use crate::rf_engine::RfEngine;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use ncollide3d::world::CollisionGroups;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyPartHandle, ColliderDesc, ColliderHandle, RigidBodyDesc};
use nphysics3d::world::World;

pub struct Platform {
    body: BodyPartHandle,
    node: SceneNode,
    collider: ColliderHandle,
    // TODO - Box<trait Engine> for list
    age: AgEngine,
    rfe_q0: RfEngine,
    rfe_q1: RfEngine,
    rfe_q2: RfEngine,
    rfe_q3: RfEngine,
}

impl Part for Platform {
    fn part_desc() -> PartDesc {
        PartDesc {
            size: Vector3::new(2.0, 0.05, 1.0),
            mass: 5.0,
            density: 1.0,
            //angular_inertia: 3.0,
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
        // TODO - rigidBody/part/parent
        //world.body(self.body).unwrap().part(0).unwrap().position()
    }

    fn velocity(&self, world: &World<f32>) -> Velocity<f32> {
        let body = world.collider(self.collider).unwrap().body();
        world.body(body).unwrap().part(0).unwrap().velocity()
    }
}

impl Platform {
    pub fn new(translation: Vector3<f32>, world: &mut World<f32>, window: &mut Window) -> Self {
        // TODO - configs
        let mut group = CollisionGroups::new();
        group.set_membership(&[0]);
        group.set_whitelist(&[0]);
        //group.set_blacklist(&[1]);

        let platform_part = Self::part_desc();
        let collider = Self::collider_desc().collision_groups(group);

        let body = RigidBodyDesc::new()
            .translation(translation)
            .angular_inertia(Matrix3::from_diagonal_element(3.0))
            .mass(platform_part.mass)
            .name("platform".to_string())
            //.gravity_enabled(false)
            .collider(&collider)
            .build(world);

        let platform_handle = body.part_handle();

        let age_translation = Vector3::zeros();
        let age = AgEngine::new(platform_handle, age_translation, world, window);

        let rfe_q0_part_translation =
            Vector3::new(platform_part.size.x / 2.0, 0.0, platform_part.size.z / 2.0);
        let rfe_q0 = RfEngine::new(platform_handle, rfe_q0_part_translation, world, window);

        let rfe_q1_part_translation =
            Vector3::new(-platform_part.size.x / 2.0, 0.0, platform_part.size.z / 2.0);
        let rfe_q1 = RfEngine::new(platform_handle, rfe_q1_part_translation, world, window);

        let rfe_q2_part_translation = Vector3::new(
            -platform_part.size.x / 2.0,
            0.0,
            -platform_part.size.z / 2.0,
        );
        let rfe_q2 = RfEngine::new(platform_handle, rfe_q2_part_translation, world, window);

        let rfe_q3_part_translation =
            Vector3::new(platform_part.size.x / 2.0, 0.0, -platform_part.size.z / 2.0);
        let rfe_q3 = RfEngine::new(platform_handle, rfe_q3_part_translation, world, window);

        let color = Point3::new(1.0, 0.3215, 0.3215);
        let collider = world
            .collider_world()
            .body_part_colliders(platform_handle)
            .next()
            .unwrap();
        let platform_collider = collider.handle();
        let platform_node = build_scene_node(&platform_part, collider, color, window);

        Platform {
            body: platform_handle,
            node: platform_node,
            collider: platform_collider,
            age,
            rfe_q0,
            rfe_q1,
            rfe_q2,
            rfe_q3,
        }
    }

    pub fn update(&mut self, world: &World<f32>, win: &mut Window) {
        update_scene_node(self.collider, world, &mut self.node);

        self.age.update(world);
        for rfe in &mut self.rf_engines_mut() {
            rfe.update(world);
        }

        self.draw_velocity_vector(world, win);

        self.age.draw_force_vector(world, win);
        for rfe in &self.rf_engines() {
            rfe.draw_force_vector(world, win);
        }
    }

    pub fn set_control_setpoints(&mut self, setpoints: &ControlSetpoints, world: &mut World<f32>) {
        self.age.set_force(
            Force::linear(Vector3::new(0.0, setpoints.ag_force, 0.0)),
            world,
        );

        let mut rfe_forces = [Force::<f32>::zero(); 4];
        for f in &mut rfe_forces {
            f.linear.x += setpoints.long_force;
            f.linear.z += setpoints.lat_force;
        }

        self.rf_engines_mut()
            .iter_mut()
            .zip(rfe_forces.iter())
            .for_each(|(rfe, f)| rfe.set_force(*f, world));
    }

    fn draw_velocity_vector(&self, world: &World<f32>, win: &mut Window) {
        // TODO - configs
        let color = Point3::new(0.0, 1.0, 0.0);
        let scale = 0.5;
        let surface_offset = 0.025;

        let mut a = self.position(world).translation.vector;
        a.y += surface_offset + Self::part_desc().size.y / 2.0;
        let b = a + (self.velocity(world).linear * scale);

        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
    }

    fn rf_engines(&self) -> [&RfEngine; 4] {
        [&self.rfe_q0, &self.rfe_q1, &self.rfe_q2, &self.rfe_q3]
    }

    fn rf_engines_mut(&mut self) -> [&mut RfEngine; 4] {
        [
            &mut self.rfe_q0,
            &mut self.rfe_q1,
            &mut self.rfe_q2,
            &mut self.rfe_q3,
        ]
    }
}
