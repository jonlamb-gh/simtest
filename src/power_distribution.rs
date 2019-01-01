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
    control: Control,
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

#[derive(Debug, Clone, Copy)]
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
        PowerDistribution {
            control: Control {
                roll_comp: 0.0,
                pitch_comp: 0.0,
                e0: 0.0,
                e1: 0.0,
                e2: 0.0,
                e3: 0.0,
            },
            engines,
        }
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

    pub fn get_control(&self) -> &Control {
        &self.control
    }

    // TODO - update control here
    fn set_force(&mut self, engine: Engine, force: Force<f32>, world: &mut World<f32>) {
        if let Some(e) = self.engines.get_mut(&engine) {
            e.set_force(force, world);
        }
    }

    pub fn control_thrust(&mut self, control: &Control, world: &mut World<f32>) {
        // TODO
        self.control.roll_comp = control.roll_comp;
        self.control.pitch_comp = control.pitch_comp;

        self.control.e0 = clamp(control.e0, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e1 = clamp(control.e1, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e2 = clamp(control.e2, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e3 = clamp(control.e3, -THRUST_LIMIT, THRUST_LIMIT);

        let mut f: Force<f32> = Force::zero();

        f.linear.y = self.control.e0;
        self.set_force(Engine::E0, f, world);

        f.linear.y = self.control.e1;
        self.set_force(Engine::E1, f, world);

        f.linear.y = self.control.e2;
        self.set_force(Engine::E2, f, world);

        f.linear.y = self.control.e3;
        self.set_force(Engine::E3, f, world);
    }
}
