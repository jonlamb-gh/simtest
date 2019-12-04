use crate::base_frame::BaseFrame;
use crate::box_node::build_scene_node;
use crate::na::{Isometry3, Point3, Vector3};
use crate::rf_engine::RfEngine;
use kiss3d::window;
use kiss3d::window::Window;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::algebra::ForceType;
use nphysics3d::joint::{FixedJoint, FreeJoint};
use nphysics3d::math::Force;
use nphysics3d::object::Body;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, MultibodyDesc, RigidBodyDesc,
};

pub struct Platform {
    base_frame: BaseFrame,
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
        let mut body = RigidBodyDesc::new()
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

        Platform { base_frame }
    }

    pub fn apply_forces(
        &mut self,
        bodies: &mut DefaultBodySet<f32>,
        colliders: &DefaultColliderSet<f32>,
    ) {
        // TODO
        //        self.rfe_fr.set_force(
        //            Force::linear(Vector3::new(0.0, 5.0, 0.0)),
        //            bodies,
        //            colliders,
        //        );

        let point = Point3::new(1.0, 0.0, 1.0);

        //let co = colliders.get(self.collider).unwrap();
        //let body = bodies.get_mut(co.body()).unwrap();

        let body = bodies.get_mut(self.base_frame.body_part.0).unwrap();

        let force = Force::linear(Vector3::new(0.0, 1.0, 0.0));

        body.apply_local_force_at_local_point(
            self.base_frame.body_part.1,
            &force.linear,
            &point,
            ForceType::Force,
            true,
        );
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        self.base_frame.update(colliders);

        //        self.rf_engines_mut()
        //            .iter_mut()
        //            .for_each(|rfe| rfe.update(colliders));
    }

    pub fn draw_vectors(&mut self, colliders: &DefaultColliderSet<f32>, win: &mut Window) {
        //        self.rf_engines()
        //            .iter()
        //            .for_each(|rfe| rfe.draw_force_vector(colliders, win));
    }

    //    fn rf_engines(&mut self) -> [&RfEngine; 4] {
    //        [&self.rfe_fl, &self.rfe_fr, &self.rfe_rl, &self.rfe_rr]
    //    }
    //
    //    fn rf_engines_mut(&mut self) -> [&mut RfEngine; 4] {
    //        [
    //            &mut self.rfe_fl,
    //            &mut self.rfe_fr,
    //            &mut self.rfe_rl,
    //            &mut self.rfe_rr,
    //        ]
    //    }
}
