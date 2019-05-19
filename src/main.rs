mod ag_engine;
mod box_node;
mod config;
mod controller;
mod ground;
mod part;
mod platform;
mod rf_engine;
mod util;

use nalgebra as na;

use crate::controller::Controller;
use crate::ground::Ground;
use crate::na::{Point3, Vector3};
use crate::part::Part;
use crate::platform::Platform;
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::window::{State, Window};
use nphysics3d::world::World;

struct AppState {
    controller: Controller,
    arc_ball: ArcBall,
    world: World<f32>,
    ground: Ground,
    platform: Platform,
}

impl AppState {
    fn new(window: &mut Window) -> Self {
        let controller = Controller::new();

        let arc_ball = ArcBall::new(Point3::new(-5.0, 5.0, -5.0), Point3::new(0.0, 0.0, 0.0));

        let mut world = World::new();
        // TODO - configs
        world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        let ground = Ground::new(&mut world, window);

        let platform = Platform::new(Vector3::new(0.0, 2.0, 0.0), &mut world, window);

        world.step();

        AppState {
            controller,
            arc_ball,
            world,
            ground,
            platform,
        }
    }
}

impl State for AppState {
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut Camera>,
        Option<&mut PlanarCamera>,
        Option<&mut PostProcessingEffect>,
    ) {
        (Some(&mut self.arc_ball), None, None)
    }

    fn step(&mut self, win: &mut Window) {
        if !win.is_closed() && !win.should_close() {
            self.platform
                .set_control_setpoints(self.controller.update(), &mut self.world);

            self.world.step();

            let p = self.platform.position(&self.world);
            //self.arc_ball.set_at(Point3::new(
            //    p.translation.x,
            //    p.translation.y,
            //   p.translation.z,
            //));
            self.arc_ball.look_at(
                Point3::new(-5.0, 5.0, -5.0),
                Point3::new(p.translation.x, p.translation.y, p.translation.z),
            );

            self.ground.update(&self.world);

            self.platform.update(&self.world, win);
        }
    }
}

fn main() {
    let mut window = Window::new("SimTest");
    window.set_framerate_limit(Some(60));
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.9, 0.9, 0.9);

    let state = AppState::new(&mut window);

    window.render_loop(state)
}
