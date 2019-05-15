use nphysics3d::algebra::ForceType;
use nphysics3d::force_generator::ForceGenerator;
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, BodySet};
use nphysics3d::solver::IntegrationParameters;

pub struct ForceGen {
    body: BodyHandle,
    force: Force<f32>,
}

impl ForceGen {
    pub fn new(body: BodyHandle) -> Self {
        ForceGen {
            body,
            force: Force::zero(),
        }
    }

    pub fn force(&self) -> Force<f32> {
        self.force
    }

    pub fn set_force(&mut self, force: Force<f32>) {
        self.force = force;
    }
}

impl ForceGenerator<f32> for ForceGen {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        if bodies.contains(self.body) {
            let part = bodies.body_mut(self.body).unwrap();
            part.apply_force(0, &self.force, ForceType::AccelerationChange, false);
        }

        // If `false` is returned, the physis world will remove
        // this force generator after this call.
        true
    }
}
