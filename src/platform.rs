use crate::base_frame::BaseFrame;
use crate::box_node::build_scene_node;
use crate::controller::{Outputs, Sensors};
use crate::na::{Point3, Vector3};
use crate::rf_engine::RfEngine;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::{Isometry, Velocity};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, RigidBodyDesc,
};

pub struct Platform {
    base_frame: BaseFrame,
    rfe_fl: RfEngine,
    rfe_fr: RfEngine,
    rfe_rl: RfEngine,
    rfe_rr: RfEngine,
}

const START_POS_Y: f32 = 2.0;

impl Platform {
    pub fn new(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        window: &mut Window,
    ) -> Self {
        // TODO - configs
        //let start_y = START_POS_Y;
        let density = 1.0;
        let platform_mass = 50.0;
        // angular_inertia

        let base_half_size = BaseFrame::size() / 2.0;
        let base_shape = ShapeHandle::new(Cuboid::new(base_half_size));
        let body_collider = ColliderDesc::new(base_shape).density(density);
        let body = RigidBodyDesc::new()
            .translation(Vector3::new(0.0, START_POS_Y, 0.0))
            .mass(platform_mass)
            .gravity_enabled(false);

        let platform = body.build();
        //platform.enable_gravity(false);
        //platform.set_angular_velocity(Vector3::new(0.0, 0.0, 1.0));
        let base_handle = bodies.insert(platform);

        let body_part = BodyPartHandle(base_handle, 0);
        let co = body_collider.build(body_part);
        let margin = co.margin();
        let half_extents = base_half_size + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(1.0, 0.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let base_frame = BaseFrame::new(body_part, collider, node);

        let rfe_fl_pos = Vector3::new(base_half_size.x, 0.0, -base_half_size.z);
        let rfe_fl = RfEngine::new(rfe_fl_pos);

        let rfe_fr_pos = Vector3::new(base_half_size.x, 0.0, base_half_size.z);
        let rfe_fr = RfEngine::new(rfe_fr_pos);

        let rfe_rl_pos = Vector3::new(-base_half_size.x, 0.0, -base_half_size.z);
        let rfe_rl = RfEngine::new(rfe_rl_pos);

        let rfe_rr_pos = Vector3::new(-base_half_size.x, 0.0, base_half_size.z);
        let rfe_rr = RfEngine::new(rfe_rr_pos);

        Platform {
            base_frame,
            rfe_fl,
            rfe_fr,
            rfe_rl,
            rfe_rr,
        }
    }

    pub fn reset_all(&mut self, bodies: &mut DefaultBodySet<f32>) {
        // TODO - reset dynamics
        let body_part = self.base_frame.body_part();
        let body = bodies.rigid_body_mut(body_part.0).unwrap();

        body.set_velocity(Velocity::zero());
        body.set_position(Isometry::translation(0.0, START_POS_Y, 0.0));
    }

    pub fn sensors(&self, bodies: &DefaultBodySet<f32>) -> Sensors {
        let body_part = self.base_frame.body_part();
        let body = bodies.rigid_body(body_part.0).unwrap();
        Sensors {
            iso: *body.position(),
            vel: *body.velocity(),
        }
    }

    pub fn apply_forces(&mut self, controller_outputs: &Outputs, bodies: &mut DefaultBodySet<f32>) {
        self.rfe_fl.set_force(controller_outputs.rfe_fl_force);
        self.rfe_fr.set_force(controller_outputs.rfe_fr_force);
        self.rfe_rl.set_force(controller_outputs.rfe_rl_force);
        self.rfe_rr.set_force(controller_outputs.rfe_rr_force);

        let body_part = self.base_frame.body_part();
        let body = bodies.rigid_body_mut(body_part.0).unwrap();

        self.rf_engines()
            .iter()
            .for_each(|rfe| rfe.apply_force(body));
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        self.base_frame.update(colliders);
    }

    pub fn draw_vectors(&mut self, bodies: &DefaultBodySet<f32>, win: &mut Window) {
        let body_part = self.base_frame.body_part();
        let body = bodies.rigid_body(body_part.0).unwrap();
        let body_pos = body.position();

        self.rf_engines()
            .iter()
            .for_each(|rfe| rfe.draw_force_vector(body_pos, win));
    }

    fn rf_engines(&mut self) -> [&RfEngine; 4] {
        [&self.rfe_fl, &self.rfe_fr, &self.rfe_rl, &self.rfe_rr]
    }
}
