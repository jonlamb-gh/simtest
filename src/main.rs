use nalgebra as na;

use self::na::{Isometry3, UnitQuaternion, Vector3};
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

const COLLIDER_MARGIN: f32 = 0.01;

struct AppState {
    c: SceneNode,
    rot: UnitQuaternion<f32>,
}

impl State for AppState {
    fn step(&mut self, _: &mut Window) {
        self.c.prepend_to_local_rotation(&self.rot)
    }
}

fn main() {
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let rad = 0.1;
    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();
    let pos = Isometry3::new(Vector3::new(0.0, 0.0, 2.0), na::zero());
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);
    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry3::identity(),
        Material::default(),
    );

    let mut window = Window::new("Window");
    window.set_framerate_limit(Some(60));
    window.set_light(Light::StickToCamera);

    let mut c = window.add_cube(1.0, 1.0, 1.0);
    c.set_color(1.0, 0.0, 0.0);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
    let state = AppState { c, rot };

    window.render_loop(state)
}
