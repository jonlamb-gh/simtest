use crate::config::THRUST_LIMIT;
use crate::lag_engine::LAGEngine;
use crate::na;
use crate::na::Vector3;
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

    pub fn control_thrust(&mut self, control: &Control, world: &mut World<f32>) {
        // TODO - compensation

        self.e0.set_force(Self::clamp_force(&control.e0), world);

        /*
        self.control.e0 = clamp(control.e0, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e1 = clamp(control.e1, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e2 = clamp(control.e2, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e3 = clamp(control.e3, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e4 = clamp(control.e4, -THRUST_LIMIT, THRUST_LIMIT);
        self.control.e5 = clamp(control.e5, -THRUST_LIMIT, THRUST_LIMIT);

        let mut f: Force<f32> = Force::zero();

        f.linear.y = self.control.e0;
        self.set_force(Engine::E0, f, world);

        f.linear.y = self.control.e1;
        self.set_force(Engine::E1, f, world);

        f.linear.y = self.control.e2;
        self.set_force(Engine::E2, f, world);

        f.linear.y = self.control.e3;
        self.set_force(Engine::E3, f, world);

        f = Force::zero();
        f.linear.x = self.control.e4;
        // TODO
        f.angular.y = control.ty;
        self.set_force(Engine::E4, f, world);

        f = Force::zero();
        f.linear.z = self.control.e5;
        self.set_force(Engine::E5, f, world);
        */
    }

    fn clamp_force(f: &Force<f32>) -> Force<f32> {
        Force::linear(Vector3::new(
            clamp(f.linear.x, -THRUST_LIMIT, THRUST_LIMIT),
            clamp(f.linear.y, -THRUST_LIMIT, THRUST_LIMIT),
            clamp(f.linear.z, -THRUST_LIMIT, THRUST_LIMIT),
        ))
    }
}
