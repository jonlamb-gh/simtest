use nalgebra as na;

use crate::controller::Controller;
use crate::ground::Ground;
use crate::inputs::Inputs;
use crate::na::{Point3, Vector3};
use crate::platform::Platform;
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::window::{State, Window};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

mod base_frame;
mod box_node;
mod config;
mod controller;
mod ground;
mod inputs;
mod platform;
mod rf_engine;
mod util;

struct AppState {
    inputs: Inputs,
    controller: Controller,
    arc_ball: ArcBall,
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    constraints: DefaultJointConstraintSet<f32>,
    forces: DefaultForceGeneratorSet<f32>,
    ground: Ground,
    platform: Platform,
}

impl AppState {
    fn new(window: &mut Window) -> Self {
        let inputs = Inputs::new();
        let controller = Controller::new();

        let arc_ball = ArcBall::new(Point3::new(-5.0, 5.0, -5.0), Point3::new(0.0, 0.0, 0.0));

        // TODO - configs
        let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0));
        let geometrical_world = DefaultGeometricalWorld::new();
        let mut bodies = DefaultBodySet::new();
        let mut colliders = DefaultColliderSet::new();
        let joint_constraints = DefaultJointConstraintSet::new();
        let force_generators = DefaultForceGeneratorSet::new();

        let ground = Ground::new(&mut bodies, &mut colliders, window);

        let platform = Platform::new(&mut bodies, &mut colliders, window);

        let mut app = AppState {
            inputs,
            controller,
            arc_ball,
            mechanical_world,
            geometrical_world,
            bodies,
            colliders,
            constraints: joint_constraints,
            forces: force_generators,
            ground,
            platform,
        };

        // panics when apply forces if initial step isn't done?
        // mechanical_world.maintain()
        app.mechanical_world.step(
            &mut app.geometrical_world,
            &mut app.bodies,
            &mut app.colliders,
            &mut app.constraints,
            &mut app.forces,
        );

        app
    }
}

impl State for AppState {
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.arc_ball), None, None)
    }

    fn step(&mut self, win: &mut Window) {
        if !win.is_closed() && !win.should_close() {
            self.inputs.update();

            let sensors = self.platform.sensors(&self.bodies);

            let outputs = self.controller.update(&self.inputs.set_points, &sensors);

            self.platform.apply_forces(outputs, &mut self.bodies);

            self.mechanical_world.step(
                &mut self.geometrical_world,
                &mut self.bodies,
                &mut self.colliders,
                &mut self.constraints,
                &mut self.forces,
            );

            // TODO
            if !self.inputs.aux.view_mode {
                self.arc_ball.set_at(Point3::new(
                    sensors.iso.translation.x,
                    sensors.iso.translation.y,
                    sensors.iso.translation.z,
                ));
            } else {
                //self.arc_ball.look_at(
                //Point3::new(-5.0, 5.0, -5.0),
                //Point3::new(
                //    sensors.iso.translation.x,
                //    sensors.iso.translation.y,
                //    sensors.iso.translation.z,
                //),
                //);
            }

            self.ground.update(&self.colliders);

            self.platform.update(&self.colliders);

            self.platform.draw_vectors(&self.bodies, win);

            if self.inputs.aux.reset_all {
                self.platform.reset_all(&mut self.bodies);
                self.controller.reset();
            }
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
