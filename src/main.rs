use nalgebra as na;

mod box_node;
mod config;
mod lag_engine;
mod node;
mod platform;

use crate::box_node::BoxNode;
use crate::config::COLLIDER_MARGIN;
use crate::na::{Isometry3, Point3, Vector3};
use crate::node::Node;
use crate::platform::{Engine, Platform};
use gilrs::{Axis, Button, Gilrs};
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::window::{State, Window};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::Force;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::world::World;

struct AppState {
    cm: Gilrs,
    arc_ball: ArcBall,
    world: World<f32>,
    ground: Node,
    platform: Platform,
}

impl AppState {
    pub fn new(window: &mut Window) -> Self {
        let cm = Gilrs::new().unwrap();

        let arc_ball = ArcBall::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));

        let mut world = World::new();
        world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        let ground_size = Vector3::new(50.0, 1.0, 50.0);
        let ground_shape =
            ShapeHandle::new(Cuboid::new(ground_size - Vector3::repeat(COLLIDER_MARGIN)));
        let ground_pos = Isometry3::new(Vector3::y() * -ground_size.y, na::zero());

        let ground_ch = world.add_collider(
            COLLIDER_MARGIN,
            ground_shape.clone(),
            BodyHandle::ground(),
            ground_pos,
            Material::default(),
        );

        let object = ground_ch;

        let color = Point3::new(0.4, 0.4, 0.4);

        let shape = ground_shape.as_shape::<Cuboid<f32>>().unwrap();

        let margin = world.collider(object).unwrap().data().margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;
        let rz = shape.half_extents().z + margin;
        let delta = na::one();

        let ground = Node::Box(BoxNode::new(
            object, &mut world, delta, rx, ry, rz, color, window,
        ));

        let platform = Platform::new(Vector3::new(0.0, 5.0, 0.0), window, &mut world);

        AppState {
            cm,
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
            while let Some(ev) = self.cm.next_event() {
                self.cm.update(&ev);
                // TODO
                // Do other things with event
            }

            let max = 50.0;
            let max_diff = 0.6;

            // TODO - move to state
            let mut thrust_left = 0.0;
            let mut thrust_right = 0.0;
            let mut balance_left = 0.0;
            let mut balance_right = 0.0;

            if let Some(input) = self.cm.gamepad(0) {
                if input.is_pressed(Button::West) {
                    // TODO - reset platform
                }

                if let Some(btn) = input.button_data(Button::LeftTrigger2) {
                    thrust_left = max * btn.value();
                }

                if let Some(btn) = input.button_data(Button::RightTrigger2) {
                    thrust_right = max * btn.value();
                }

                balance_left = input.value(Axis::LeftStickY);
                balance_right = input.value(Axis::RightStickY);

                /*
                let state = input.state();
                for (code, axis) in state.axes() {
                    println!("{:#?}", input.axis_or_btn_name(code));
                }
                */

                /*
                let state = input.state();
                for (code, btn) in state.buttons() {
                    println!("{:#?}", input.axis_or_btn_name(code));
                }
                */
            }

            let scale_left = map_range((-1.0, 1.0), (-max_diff, max_diff), balance_left);

            let scale_right = map_range((-1.0, 1.0), (-max_diff, max_diff), balance_right);

            let diff_left = scale_left * thrust_left;
            let diff_right = scale_right * thrust_right;

            // TODO - re-order
            // left is e2, e3
            // right is e0, e1
            let f_e0 = Force::new(
                Vector3::new(0.0, thrust_right - diff_right, 0.0),
                na::zero(),
            );
            let f_e1 = Force::new(
                Vector3::new(0.0, thrust_right + diff_right, 0.0),
                na::zero(),
            );
            let f_e2 = Force::new(Vector3::new(0.0, thrust_left - diff_left, 0.0), na::zero());
            let f_e3 = Force::new(Vector3::new(0.0, thrust_left + diff_left, 0.0), na::zero());

            self.platform.set_force(Engine::E0, f_e0, &mut self.world);
            self.platform.set_force(Engine::E1, f_e1, &mut self.world);
            self.platform.set_force(Engine::E2, f_e2, &mut self.world);
            self.platform.set_force(Engine::E3, f_e3, &mut self.world);

            self.world.step();

            self.ground.update(&self.world);
            self.platform.update(&self.world);
        }
    }
}

fn map_range(from_range: (f32, f32), to_range: (f32, f32), s: f32) -> f32 {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}

fn main() {
    let mut window = Window::new("Window");
    window.set_framerate_limit(Some(60));
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.9, 0.9, 0.9);

    let state = AppState::new(&mut window);

    window.render_loop(state)
}
