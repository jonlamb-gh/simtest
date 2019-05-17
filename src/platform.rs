use crate::ag_engine::AgEngine;
use crate::box_node::{build_scene_node, update_scene_node};
use crate::control_setpoints::ControlSetpoints;
use crate::force_gen::ForceGen;
use crate::na::{self, Isometry3, Point3, Vector3};
use crate::part::{Part, PartDesc};
use crate::rf_engine::RfEngine;
use kiss3d::scene::SceneNode;
use kiss3d::window;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::FreeJoint;
use nphysics3d::math::Force;
use nphysics3d::math::Velocity;
use nphysics3d::object::{
    BodyHandle, BodyPartHandle, Collider, ColliderDesc, ColliderHandle, Multibody, MultibodyDesc,
};
use nphysics3d::world::World;

pub struct Platform {
    body: BodyHandle,
    node: SceneNode,
    collider: ColliderHandle,
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
        //world.body(self.body).unwrap().part(0).unwrap().velocity()

        let body = world.collider(self.collider).unwrap().body();
        world.body(body).unwrap().part(0).unwrap().velocity()
    }
}

impl Platform {
    pub fn new(
        world: &mut World<f32>,
        translation: Vector3<f32>,
        window: &mut window::Window,
    ) -> Self {
        // Build all of the mbody links/etc into the world,
        // then iterate through the colliders and construct
        // the user facing objects/parts

        let platform_part = Self::part_desc();
        let collider = Self::collider_desc();

        let root_joint = FreeJoint::new(Isometry3::new(translation, na::zero()));
        //let root_joint = FreeJoint::new(Isometry3::new(translation,
        // Vector3::new(0.0, 0.0, 0.4)));

        let body = MultibodyDesc::new(root_joint)
            .mass(platform_part.mass)
            .collider(&collider)
            .name("platform".to_string());

        let collider = AgEngine::collider_desc();
        let part_translation = Vector3::zeros();
        let body = AgEngine::build_link("ag_engine".to_string(), part_translation, &collider, body);

        let collider = RfEngine::collider_desc();
        let rfe_q0_part_translation =
            Vector3::new(platform_part.size.x / 2.0, 0.0, platform_part.size.z / 2.0);
        let body = RfEngine::build_link(
            "rf_engine.q0".to_string(),
            rfe_q0_part_translation,
            &collider,
            body,
        );

        let collider = RfEngine::collider_desc();
        let rfe_q1_part_translation =
            Vector3::new(-platform_part.size.x / 2.0, 0.0, platform_part.size.z / 2.0);
        let body = RfEngine::build_link(
            "rf_engine.q1".to_string(),
            rfe_q1_part_translation,
            &collider,
            body,
        );

        let collider = RfEngine::collider_desc();
        let rfe_q2_part_translation = Vector3::new(
            -platform_part.size.x / 2.0,
            0.0,
            -platform_part.size.z / 2.0,
        );
        let body = RfEngine::build_link(
            "rf_engine.q2".to_string(),
            rfe_q2_part_translation,
            &collider,
            body,
        );

        let collider = RfEngine::collider_desc();
        let rfe_q3_part_translation =
            Vector3::new(platform_part.size.x / 2.0, 0.0, -platform_part.size.z / 2.0);
        let body = RfEngine::build_link(
            "rf_engine.q3".to_string(),
            rfe_q3_part_translation,
            &collider,
            body,
        );

        // Build the multibody into the world
        let mbody = body.build(world);

        let platform_handle = Self::get_body_part("platform", &mbody);
        let age_handle = Self::get_body_part("ag_engine", &mbody);
        let rfe_q0_handle = Self::get_body_part("rf_engine.q0", &mbody);
        let rfe_q1_handle = Self::get_body_part("rf_engine.q1", &mbody);
        let rfe_q2_handle = Self::get_body_part("rf_engine.q2", &mbody);
        let rfe_q3_handle = Self::get_body_part("rf_engine.q3", &mbody);

        let color = Point3::new(1.0, 0.3215, 0.3215);
        let collider = Self::get_collider(platform_handle, world);
        let platform_body = collider.body();
        let platform_collider = collider.handle();
        let platform_node = build_scene_node(&platform_part, collider, color, window);

        let color = Point3::new(0.0, 0.6352, 1.0);
        let collider = Self::get_collider(age_handle, world);
        let body = collider.body();
        let handle = collider.handle();
        let node = build_scene_node(&AgEngine::part_desc(), collider, color, window);
        let force_gen = world.add_force_generator(ForceGen::new(body, None, false, 0));
        let age = AgEngine::new(body, handle, node, force_gen);

        let color = Point3::new(0.0, 1.0, 0.1019);
        let collider = Self::get_collider(rfe_q0_handle, world);
        let body = collider.body();
        let handle = collider.handle();
        let node = build_scene_node(&RfEngine::part_desc(), collider, color, window);
        let force_gen = world.add_force_generator(ForceGen::new(
            body,
            Some(Point3::from(rfe_q0_part_translation)),
            true,
            //false,
            1,
        ));
        let rfe_q0 = RfEngine::new(body, handle, node, force_gen);

        let color = Point3::new(0.0, 1.0, 0.1019);
        let collider = Self::get_collider(rfe_q1_handle, world);
        let body = collider.body();
        let handle = collider.handle();
        let node = build_scene_node(&RfEngine::part_desc(), collider, color, window);
        let force_gen = world.add_force_generator(ForceGen::new(
            body,
            Some(Point3::from(rfe_q1_part_translation)),
            true,
            1,
        ));
        let rfe_q1 = RfEngine::new(body, handle, node, force_gen);

        let color = Point3::new(0.0, 1.0, 0.1019);
        let collider = Self::get_collider(rfe_q2_handle, world);
        let body = collider.body();
        let handle = collider.handle();
        let node = build_scene_node(&RfEngine::part_desc(), collider, color, window);
        let force_gen = world.add_force_generator(ForceGen::new(
            body,
            Some(Point3::from(rfe_q2_part_translation)),
            //None,
            true,
            //false,
            3,
        ));
        let rfe_q2 = RfEngine::new(body, handle, node, force_gen);

        let color = Point3::new(0.0, 1.0, 0.1019);
        let collider = Self::get_collider(rfe_q3_handle, world);
        let body = collider.body();
        let handle = collider.handle();
        let node = build_scene_node(&RfEngine::part_desc(), collider, color, window);
        let force_gen = world.add_force_generator(ForceGen::new(
            body,
            Some(Point3::from(rfe_q3_part_translation)),
            true,
            3,
        ));
        let rfe_q3 = RfEngine::new(body, handle, node, force_gen);

        Platform {
            body: platform_body,
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
        // TODO

        self.age.set_force(
            Force::linear(Vector3::new(0.0, setpoints.ag_force, 0.0)),
            world,
        );

        // Rf engine relative forces
        let mut rfe_forces = [Force::<f32>::zero(); 4];

        //        for f in &mut rfe_forces {
        //            f.linear.x += setpoints.long_force;
        //            f.linear.z += setpoints.lat_force;
        //        }

        //rfe_forces[2].linear.x = 0.0;
        rfe_forces[2].linear.z = -3.0;
        //rfe_forces[2].linear.y = -10.0;

        rfe_forces[0].linear.z = 3.0;

        self.rfe_q0.set_force(rfe_forces[0], world);
        self.rfe_q2.set_force(rfe_forces[2], world);

        //        self.rf_engines_mut()
        //            .iter_mut()
        //            .zip(rfe_forces.iter())
        //            .for_each(|(rfe, f)| rfe.set_force(*f, world));
    }

    fn get_body_part(name: &str, mbody: &Multibody<f32>) -> BodyPartHandle {
        mbody.links_with_name(name).next().unwrap().part_handle()
    }

    fn get_collider<'a>(part_handle: BodyPartHandle, world: &'a World<f32>) -> &'a Collider<f32> {
        world
            .collider_world()
            .body_part_colliders(part_handle)
            .next()
            .unwrap()
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
