use crate::config::THRUST_LIMIT;
use crate::lag_engine::LAGEngine;
use crate::na;
use crate::na::{Isometry3, Vector3};
use crate::util::clamp;
use nphysics3d::math::Force;
use nphysics3d::world::World;

pub struct PowerDistribution {
    /// Quadrant 0, Rear left
    e0: LAGEngine,
    /// Quadrant 1, Front left
    e1: LAGEngine,
    /// Quadrant 2, Front right
    e2: LAGEngine,
    /// Quadrant 3, Rear right
    e3: LAGEngine,
}

#[derive(Debug, Clone, Copy)]
pub struct Control {
    pub att_comp: Vector3<f32>,
    pub e0: Force<f32>,
    pub e1: Force<f32>,
    pub e2: Force<f32>,
    pub e3: Force<f32>,
}

#[derive(Debug, Clone, Copy)]
pub struct EnginePositions {
    pub e0: Isometry3<f32>,
    pub e1: Isometry3<f32>,
    pub e2: Isometry3<f32>,
    pub e3: Isometry3<f32>,
}

impl Control {
    pub fn new() -> Self {
        Control {
            att_comp: na::zero(),
            e0: Force::zero(),
            e1: Force::zero(),
            e2: Force::zero(),
            e3: Force::zero(),
        }
    }
}

impl PowerDistribution {
    pub fn new(e0: LAGEngine, e1: LAGEngine, e2: LAGEngine, e3: LAGEngine) -> Self {
        PowerDistribution { e0, e1, e2, e3 }
    }

    pub fn engine_positions(&self, world: &World<f32>) -> EnginePositions {
        EnginePositions {
            e0: self.e0.position(world),
            e1: self.e1.position(world),
            e2: self.e2.position(world),
            e3: self.e3.position(world),
        }
    }

    pub fn update(&mut self, world: &World<f32>) {
        self.e0.update(world);
        self.e1.update(world);
        self.e2.update(world);
        self.e3.update(world);
    }

    pub fn reset(&mut self, world: &mut World<f32>) {
        self.e0.set_force(Force::zero(), world);
        self.e1.set_force(Force::zero(), world);
        self.e2.set_force(Force::zero(), world);
        self.e3.set_force(Force::zero(), world);
    }

    pub fn set_control(&mut self, control: &Control, world: &mut World<f32>) {
        self.e0.set_force(Self::clamp_force(&control.e0), world);
        self.e1.set_force(Self::clamp_force(&control.e1), world);
        self.e2.set_force(Self::clamp_force(&control.e2), world);
        self.e3.set_force(Self::clamp_force(&control.e3), world);
    }

    fn clamp_force(f: &Force<f32>) -> Force<f32> {
        Force::linear(Vector3::new(
            clamp(f.linear.x, -THRUST_LIMIT, THRUST_LIMIT),
            clamp(f.linear.y, -THRUST_LIMIT, THRUST_LIMIT),
            clamp(f.linear.z, -THRUST_LIMIT, THRUST_LIMIT),
        ))
    }
}
