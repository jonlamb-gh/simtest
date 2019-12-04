use crate::base_frame::BaseFrame;
use crate::box_node::build_scene_node;
use crate::na::{Point3, Vector3};
use crate::rf_engine::RfEngine;
use kiss3d::window;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::Force;
use nphysics3d::object::Body;
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

impl Platform {
    pub fn new(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        window: &mut window::Window,
    ) -> Self {
        // TODO - configs
        let start_y = 2.0;
        let density = 1.0;
        let platform_mass = 10.0;
        // angular_inertia

        let base_shape = ShapeHandle::new(Cuboid::new(BaseFrame::size()));
        let body_collider = ColliderDesc::new(base_shape).density(density);
        let body = RigidBodyDesc::new()
            .translation(Vector3::new(0.0, start_y, 0.0))
            .mass(platform_mass)
            .gravity_enabled(false);

        let mut platform = body.build();
        platform.enable_gravity(false);
        let base_handle = bodies.insert(platform);

        let body_part = BodyPartHandle(base_handle, 0);
        let co = body_collider.build(body_part);
        let margin = co.margin();
        let half_extents = (BaseFrame::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(1.0, 0.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let base_frame = BaseFrame::new(body_part, collider, node);

        let rfe_fl_pos = Vector3::new(BaseFrame::size().x / 2.0, 0.0, BaseFrame::size().z / -2.0);
        let rfe_fl = RfEngine::new(rfe_fl_pos);

        let rfe_fr_pos = Vector3::new(BaseFrame::size().x / 2.0, 0.0, BaseFrame::size().z / 2.0);
        let rfe_fr = RfEngine::new(rfe_fr_pos);

        let rfe_rl_pos = Vector3::new(BaseFrame::size().x / -2.0, 0.0, BaseFrame::size().z / -2.0);
        let rfe_rl = RfEngine::new(rfe_rl_pos);

        let rfe_rr_pos = Vector3::new(BaseFrame::size().x / -2.0, 0.0, BaseFrame::size().z / 2.0);
        let rfe_rr = RfEngine::new(rfe_rr_pos);

        Platform {
            base_frame,
            rfe_fl,
            rfe_fr,
            rfe_rl,
            rfe_rr,
        }
    }

    pub fn apply_forces(
        &mut self,
        bodies: &mut DefaultBodySet<f32>,
        _colliders: &DefaultColliderSet<f32>,
    ) {
        // TODO
        let force = Force::linear(Vector3::new(0.0, 1.0, 0.0));
        self.rfe_fr.set_force(force);

        let body_part = *self.base_frame.body_part();
        self.rf_engines()
            .iter()
            .for_each(|rfe| rfe.apply_force(&body_part, bodies));
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        self.base_frame.update(colliders);
    }

    pub fn draw_vectors(&mut self, colliders: &DefaultColliderSet<f32>, win: &mut Window) {
        let body_pos = colliders
            .get(*self.base_frame.collider())
            .unwrap()
            .position();

        self.rf_engines()
            .iter()
            .for_each(|rfe| rfe.draw_force_vector(body_pos, win));
    }

    fn rf_engines(&mut self) -> [&RfEngine; 4] {
        [&self.rfe_fl, &self.rfe_fr, &self.rfe_rl, &self.rfe_rr]
    }
}
