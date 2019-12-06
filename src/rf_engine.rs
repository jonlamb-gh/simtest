use crate::na::{Point3, Vector3};
use kiss3d::window::Window;
use nphysics3d::algebra::ForceType;
use nphysics3d::math::{Force, Isometry};
use nphysics3d::object::{Body, RigidBody};

pub struct RfEngine {
    pos_wrt_parent: Vector3<f32>,
    force: Force<f32>,
}

impl RfEngine {
    pub fn new(pos_wrt_parent: Vector3<f32>) -> Self {
        RfEngine {
            pos_wrt_parent,
            force: Force::zero(),
        }
    }

    //pub fn size() -> Vector3<f32> {
    //    Vector3::new(0.1, 0.14, 0.1)
    //}

    pub fn pos_wrt_parent(&self) -> &Vector3<f32> {
        &self.pos_wrt_parent
    }

    pub fn force(&self) -> &Force<f32> {
        &self.force
    }

    pub fn set_force(&mut self, local_force: Force<f32>) {
        self.force = local_force;
    }

    pub fn apply_force(&self, body: &mut RigidBody<f32>) {
        let point = Point3::from(*self.pos_wrt_parent());
        body.apply_local_force_at_local_point(
            0,
            &self.force.linear,
            &point,
            ForceType::Force,
            true,
        );
    }

    pub fn draw_force_vector(&self, body_position: &Isometry<f32>, win: &mut Window) {
        // TODO - configs
        let color = Point3::new(0.0, 0.0, 1.0);
        let scale = 0.5;

        let aligned_force = body_position
            .rotation
            .transform_vector(&self.force().linear);
        let aligned_pos = body_position
            .rotation
            .transform_vector(&self.pos_wrt_parent());

        let a = body_position.translation.vector + aligned_pos;
        let b = a + (aligned_force * scale);

        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
    }
}
