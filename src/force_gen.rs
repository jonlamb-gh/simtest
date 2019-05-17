use crate::na::{self, Isometry3, Point3, Vector3};
use nphysics3d::algebra::ForceType;
use nphysics3d::force_generator::ForceGenerator;
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, BodySet};
use nphysics3d::solver::IntegrationParameters;

pub struct ForceGen {
    body: BodyHandle,
    force: Force<f32>,
    local_point: Option<Point3<f32>>,
    local_force: bool,
    idx: usize,
}

impl ForceGen {
    pub fn new(
        body: BodyHandle,
        local_point: Option<Point3<f32>>,
        local_force: bool,
        idx: usize,
    ) -> Self {
        ForceGen {
            body,
            force: Force::zero(),
            local_point,
            local_force,
            idx,
        }
    }

    pub fn force(&self) -> Force<f32> {
        self.force
    }

    pub fn set_force(&mut self, force: Force<f32>) {
        self.force = force;
    }

    //pub fn is_local(&self) -> bool
}

impl ForceGenerator<f32> for ForceGen {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        if bodies.contains(self.body) {
            let part = bodies.body_mut(self.body).unwrap();

            if self.local_force == true {
                if let Some(lp) = self.local_point {
                    part.apply_local_force_at_local_point(
                        0,
                        &self.force.linear,
                        &lp,
                        ForceType::Force,
                        //ForceType::VelocityChange,
                        //ForceType::Impulse,
                        false,
                    );
                } else {
                    part.apply_local_force(0, &self.force, ForceType::Force, false);
                }
            } else {
                if let Some(lp) = self.local_point {
                    part.apply_force_at_local_point(
                        0,
                        &self.force.linear,
                        &lp,
                        ForceType::Force,
                        false,
                    );
                } else {
                    part.apply_force(0, &self.force, ForceType::Force, false);
                }
            }
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}
