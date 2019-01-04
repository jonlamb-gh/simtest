use nalgebra as na;

mod attitude_controller;
mod box_node;
mod config;
mod lag_engine;
mod node;
mod platform;
mod power_distribution;
mod util;
mod velocity_controller;

use crate::box_node::BoxNode;
use crate::config::*;
use crate::na::geometry::UnitQuaternion;
use crate::na::{Isometry3, Point2, Point3, Vector3};
use crate::node::Node;
use crate::platform::Platform;
use crate::util::map_range;
use gilrs::{Axis, Button, Gilrs};
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::Velocity;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::world::World;

struct AppState {
    cm: Gilrs,
    arc_ball: ArcBall,
    world: World<f32>,
    ground: Node,
    platform: Platform,
    paused: bool,
}

impl AppState {
    pub fn new(window: &mut Window) -> Self {
        let cm = Gilrs::new().unwrap();

        let arc_ball = ArcBall::new(Point3::new(-10.0, 10.0, -10.0), Point3::new(0.0, 0.0, 0.0));

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
            paused: false,
        }
    }

    /// Returns velocity setpoint and attitude setpoint
    fn control_commands(&mut self) -> (Velocity<f32>, UnitQuaternion<f32>) {
        let mut vel_y_pos = 0.0;
        let mut vel_y_neg = 0.0;
        let mut vel_x = 0.0;
        let mut vel_z = 0.0;
        let mut rot_y = 0.0;

        if let Some(input) = self.cm.gamepad(0) {
            if input.is_pressed(Button::West) {
                println!("Reseting platform");
                self.platform.reset(&mut self.world);
            }
            if input.is_pressed(Button::North) {
                self.paused = true;
            }
            if input.is_pressed(Button::East) {
                self.paused = false;
            }
            if let Some(btn) = input.button_data(Button::LeftTrigger2) {
                vel_y_neg = btn.value();
            }
            if let Some(btn) = input.button_data(Button::RightTrigger2) {
                vel_y_pos = btn.value();
            }

            vel_x = input.value(Axis::RightStickY);
            vel_z = input.value(Axis::RightStickX);

            rot_y = input.value(Axis::LeftStickX);
        }

        let vel_y = map_range(
            (-1.0, 1.0),
            (-VELOCITY_LIMIT_Y, VELOCITY_LIMIT_Y),
            vel_y_pos - vel_y_neg,
        );
        let vel_x = map_range((-1.0, 1.0), (-VELOCITY_LIMIT_XZ, VELOCITY_LIMIT_XZ), vel_x);
        let vel_z = map_range((-1.0, 1.0), (-VELOCITY_LIMIT_XZ, VELOCITY_LIMIT_XZ), vel_z);
        let vel_setpoint = Velocity::linear(vel_x, vel_y, vel_z);

        let ry_setp = map_range((-1.0, 1.0), (ATT_LIMIT_Y, -ATT_LIMIT_Y), rot_y);
        let att_setpoint = UnitQuaternion::new(Vector3::new(0.0, ry_setp, 0.0));

        (vel_setpoint, att_setpoint)
    }

    // TODO - move this somewhere else
    fn draw_text(&self, win: &mut Window) {
        // TODO - configs
        let font_size = 35.0;
        let next_font = font_size + 5.0;
        let font_color = Point3::new(1.0, 1.0, 0.0);
        let mut text_pos = Point2::new(10.0, 10.0);

        let iso = self.platform.position(&self.world);
        let pos = iso.translation.vector;
        let att = iso.rotation.scaled_axis();
        let abs_vel = self.platform.velocity(&self.world);
        let rel_vel = self.platform.rel_velocity(&self.world);
        let vel_sp = self.platform.velocity_setpoint();
        let att_sp = self.platform.attitude_setpoint().scaled_axis();
        let pwr_dist_control = self.platform.power_dist_control();

        win.draw_text(
            &format!("Position: {:.3}, {:.3}, {:.3}", pos.x, pos.y, pos.z),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Attitude: {:.3}, {:.3}, {:.3}",
                att.x.to_degrees(),
                att.y.to_degrees(),
                att.z.to_degrees(),
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Absolute Velocity: {:.3}, {:.3}, {:.3}",
                abs_vel.linear.x, abs_vel.linear.y, abs_vel.linear.z
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Relative Velocity: {:.3}, {:.3}, {:.3}",
                rel_vel.linear.x, rel_vel.linear.y, rel_vel.linear.z
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Velocity Setpoint: {:.3}, {:.3}, {:.3}",
                vel_sp.linear.x, vel_sp.linear.y, vel_sp.linear.z
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Angular Velocity: {:.3}, {:.3}, {:.3}",
                abs_vel.angular.x.to_degrees(),
                abs_vel.angular.y.to_degrees(),
                abs_vel.angular.z.to_degrees(),
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Attitude Setpoint: {:.3}, {:.3}, {:.3}",
                att_sp.x.to_degrees(),
                att_sp.y.to_degrees(),
                att_sp.z.to_degrees(),
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "Att Comp: {:.3}, {:.3}, {:.3}",
                pwr_dist_control.att_comp.x.to_degrees(),
                pwr_dist_control.att_comp.y.to_degrees(),
                pwr_dist_control.att_comp.z.to_degrees(),
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "E0: {:.3}, {:.3}, {:.3}",
                pwr_dist_control.e0.linear.x,
                pwr_dist_control.e0.linear.y,
                pwr_dist_control.e0.linear.z,
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "E1: {:.3}, {:.3}, {:.3}",
                pwr_dist_control.e1.linear.x,
                pwr_dist_control.e1.linear.y,
                pwr_dist_control.e1.linear.z,
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "E2: {:.3}, {:.3}, {:.3}",
                pwr_dist_control.e2.linear.x,
                pwr_dist_control.e2.linear.y,
                pwr_dist_control.e2.linear.z,
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );

        text_pos.y += next_font;
        win.draw_text(
            &format!(
                "E3: {:.3}, {:.3}, {:.3}",
                pwr_dist_control.e3.linear.x,
                pwr_dist_control.e3.linear.y,
                pwr_dist_control.e3.linear.z,
            ),
            &text_pos,
            font_size,
            &Font::default(),
            &font_color,
        );
    }

    fn draw_engine_force_vectors(&self, win: &mut Window) {
        let pwr_dist_control = self.platform.power_dist_control();
        let engines = self.platform.engine_positions(&self.world);
        let color = Point3::new(0.0, 1.0, 0.0);
        let f_scale = 0.5;

        let a = engines.e0.translation.vector;
        let b = a + (pwr_dist_control.e0.linear * f_scale);
        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
        let a = engines.e1.translation.vector;
        let b = a + (pwr_dist_control.e1.linear * f_scale);
        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
        let a = engines.e2.translation.vector;
        let b = a + (pwr_dist_control.e2.linear * f_scale);
        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
        let a = engines.e3.translation.vector;
        let b = a + (pwr_dist_control.e3.linear * f_scale);
        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
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

            let (vel_setpoint, att_setpoint) = self.control_commands();

            if !self.paused {
                self.platform
                    .step_controls(vel_setpoint, att_setpoint, &mut self.world);

                self.world.step();

                self.ground.update(&self.world);
                self.platform.update(&self.world);
            }

            self.draw_engine_force_vectors(win);

            self.draw_text(win);
        }
    }
}

fn main() {
    let mut window = Window::new("Window");
    window.set_framerate_limit(Some(60));
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.9, 0.9, 0.9);

    let state = AppState::new(&mut window);

    window.render_loop(state)
}
