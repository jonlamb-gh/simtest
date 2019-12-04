use crate::base_frame::BaseFrame;
use crate::box_node::build_scene_node;
use crate::na::{Isometry3, Point3, Vector3};
use crate::rf_engine::RfEngine;
use kiss3d::window;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FixedJoint, FreeJoint};
use nphysics3d::math::Force;
use nphysics3d::object::Body;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, MultibodyDesc,
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
        let density = 0.3;
        let platform_mass = 1.0;
        let rfe_mass = 0.01;
        // angular_inertia

        let base_shape = ShapeHandle::new(Cuboid::new(BaseFrame::size()));
        let base_free_joint = FreeJoint::new(Isometry3::translation(0.0, start_y, 0.0));

        let rfe_shape = ShapeHandle::new(Cuboid::new(RfEngine::size()));
        let rfe_fixed_joint = FixedJoint::new(Isometry3::identity());
        let rfe_fl_pos = Vector3::new(BaseFrame::size().x / 2.0, 0.0, BaseFrame::size().z / -2.0);
        let rfe_fr_pos = Vector3::new(BaseFrame::size().x / 2.0, 0.0, BaseFrame::size().z / 2.0);
        let rfe_rl_pos = Vector3::new(BaseFrame::size().x / -2.0, 0.0, BaseFrame::size().z / -2.0);
        let rfe_rr_pos = Vector3::new(BaseFrame::size().x / -2.0, 0.0, BaseFrame::size().z / 2.0);

        // Base platform collider setup
        let body_collider = ColliderDesc::new(base_shape).density(density);
        let mut body = MultibodyDesc::new(base_free_joint)
            .name("platform".to_owned())
            .mass(platform_mass);

        // RfEngine collider setup
        let rfe_collider = ColliderDesc::new(rfe_shape).density(density);
        body.add_child(rfe_fixed_joint)
            .set_name("rfe_fl".to_owned())
            .set_mass(rfe_mass)
            .set_parent_shift(rfe_fl_pos);
        body.add_child(rfe_fixed_joint)
            .set_name("rfe_fr".to_owned())
            .set_mass(rfe_mass)
            .set_parent_shift(rfe_fr_pos);
        body.add_child(rfe_fixed_joint)
            .set_name("rfe_rl".to_owned())
            .set_mass(rfe_mass)
            .set_parent_shift(rfe_rl_pos);
        body.add_child(rfe_fixed_joint)
            .set_name("rfe_rr".to_owned())
            .set_mass(rfe_mass)
            .set_parent_shift(rfe_rr_pos);

        // Build the base platform body and child parts
        let mut platform = body.build();
        platform.enable_gravity(false);
        let base_handle = bodies.insert(platform);

        let co = body_collider.build(BodyPartHandle(base_handle, 0));
        let margin = co.margin();
        let half_extents = (BaseFrame::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(1.0, 0.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let base_frame = BaseFrame::new(collider, node);

        // RfE fl
        let body_part = BodyPartHandle(base_handle, 1);
        let co = rfe_collider.build(body_part);
        let margin = co.margin();
        let half_extents = (RfEngine::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(0.0, 1.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let rfe_fl = RfEngine::new(body_part, collider, node);

        // RfE fr
        let body_part = BodyPartHandle(base_handle, 2);
        let co = rfe_collider.build(body_part);
        let margin = co.margin();
        let half_extents = (RfEngine::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(0.0, 1.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let rfe_fr = RfEngine::new(body_part, collider, node);

        // RfE rl
        let body_part = BodyPartHandle(base_handle, 3);
        let co = rfe_collider.build(body_part);
        let margin = co.margin();
        let half_extents = (RfEngine::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(0.0, 1.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let rfe_rl = RfEngine::new(body_part, collider, node);

        // RfE rr
        let body_part = BodyPartHandle(base_handle, 4);
        let co = rfe_collider.build(body_part);
        let margin = co.margin();
        let half_extents = (RfEngine::size() / 2.0) + Vector3::repeat(margin);
        let collider = colliders.insert(co);

        let color = Point3::new(0.0, 1.0, 0.0);
        let node = build_scene_node(collider, colliders, half_extents, color, window);
        let rfe_rr = RfEngine::new(body_part, collider, node);

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
        colliders: &DefaultColliderSet<f32>,
    ) {
        // TODO
        self.rfe_fr.set_force(
            Force::linear(Vector3::new(0.0, 5.0, 0.0)),
            bodies,
            colliders,
        );
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        self.base_frame.update(colliders);

        self.rf_engines_mut()
            .iter_mut()
            .for_each(|rfe| rfe.update(colliders));
    }

    pub fn draw_vectors(&mut self, colliders: &DefaultColliderSet<f32>, win: &mut Window) {
        self.rf_engines()
            .iter()
            .for_each(|rfe| rfe.draw_force_vector(colliders, win));
    }

    fn rf_engines(&mut self) -> [&RfEngine; 4] {
        [&self.rfe_fl, &self.rfe_fr, &self.rfe_rl, &self.rfe_rr]
    }

    fn rf_engines_mut(&mut self) -> [&mut RfEngine; 4] {
        [
            &mut self.rfe_fl,
            &mut self.rfe_fr,
            &mut self.rfe_rl,
            &mut self.rfe_rr,
        ]
    }
}
