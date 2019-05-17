mod ag_engine;
mod box_node;
mod control_setpoints;
mod force_gen;
mod ground;
mod part;
mod platform;
mod rf_engine;

use nalgebra as na;

use crate::control_setpoints::ControlSetpoints;
use crate::ground::Ground;
use crate::na::{Point3, Vector3};
use crate::platform::Platform;
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::window::{State, Window};
use nphysics3d::world::World;
//use crate::part::Part;

struct AppState {
    arc_ball: ArcBall,
    world: World<f32>,
    ground: Ground,
    platform: Platform,
    setpoints: ControlSetpoints,
}

impl AppState {
    fn new(window: &mut Window) -> Self {
        let arc_ball = ArcBall::new(Point3::new(-5.0, 5.0, -5.0), Point3::new(0.0, 0.0, 0.0));

        let mut world = World::new();
        // TODO - configs
        world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        let ground = Ground::new(&mut world, window);

        let platform = Platform::new(&mut world, Vector3::new(0.0, 2.0, 0.0), window);

        world.step();

        AppState {
            arc_ball,
            world,
            ground,
            platform,
            setpoints: ControlSetpoints::new(),
        }
    }

    fn update_setpoints(&mut self) {
        // TODO
        //self.setpoints.ag_force = 9.81;
        self.setpoints.ag_force = 6.0;

        //self.setpoints.long_force = 5.0;

        //self.setpoints.lat_force = 5.0;
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
            self.update_setpoints();

            self.platform
                .set_control_setpoints(&self.setpoints, &mut self.world);

            //            for b in self.world.bodies_mut() {
            //                b.activate();
            //            }

            self.world.step();

            self.ground.update(&self.world);

            self.platform.update(&self.world, win);

            //println!("{:#?}", self.platform.velocity(&self.world));
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
