// TODO
// - roll/pitch compensation
// like here:
// https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/power_distribution_stock.c#L74

use crate::config::THRUST_LIMIT;
use crate::lag_engine::LAGEngine;
use crate::util::clamp;
use nphysics3d::math::Force;
use nphysics3d::world::World;
use std::collections::HashMap;

pub struct PowerDistribution {
    engines: HashMap<Engine, LAGEngine>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Engine {
    /// Quadrant 0
    E0,
    /// Quadrant 1
    E1,
    /// Quadrant 2
    E2,
    /// Quadrant 3
    E3,
}

pub struct Control {
    pub roll_comp: f32,
    pub pitch_comp: f32,
    pub e0: f32,
    pub e1: f32,
    pub e2: f32,
    pub e3: f32,
}

impl PowerDistribution {
    pub fn new(engines: HashMap<Engine, LAGEngine>) -> Self {
        PowerDistribution { engines }
    }

    pub fn update(&mut self, world: &World<f32>) {
        for (_, eng) in self.engines.iter_mut() {
            eng.update(world);
        }
    }

    pub fn reset(&mut self, world: &mut World<f32>) {
        for (_, eng) in self.engines.iter_mut() {
            eng.set_force(Force::zero(), world);
        }
    }

    pub fn set_force(&mut self, engine: Engine, force: Force<f32>, world: &mut World<f32>) {
        if let Some(e) = self.engines.get_mut(&engine) {
            e.set_force(force, world);
        }
    }

    pub fn control_thrust(&mut self, control: &Control, world: &mut World<f32>) {
        // TODO

        let mut f: Force<f32> = Force::zero();

        f.linear.y = clamp(control.e0, -THRUST_LIMIT, THRUST_LIMIT);
        self.set_force(Engine::E0, f, world);

        f.linear.y = clamp(control.e1, -THRUST_LIMIT, THRUST_LIMIT);
        self.set_force(Engine::E1, f, world);

        f.linear.y = clamp(control.e2, -THRUST_LIMIT, THRUST_LIMIT);
        self.set_force(Engine::E2, f, world);

        f.linear.y = clamp(control.e3, -THRUST_LIMIT, THRUST_LIMIT);
        self.set_force(Engine::E3, f, world);
    }
}
