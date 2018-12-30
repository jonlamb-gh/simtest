use nalgebra as na;

mod box_node;
mod node;

use crate::box_node::BoxNode;
use crate::na::{Isometry3, Point3, Vector3};
use crate::node::Node;
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::window::{State, Window};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

const COLLIDER_MARGIN: f32 = 0.01;

struct AppState {
    arc_ball: ArcBall,
    world: World<f32>,
    ground: Node,
    actor: Node,
}

impl AppState {
    pub fn new(window: &mut Window) -> Self {
        let arc_ball = ArcBall::new(Point3::new(10.0, 10.0, 10.0), Point3::new(0.0, 0.0, 0.0));

        let mut world = World::new();
        world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        let ground_size = Vector3::new(15.0, 1.0, 15.0);
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

        let cube_rad = 1.0;
        let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(cube_rad - COLLIDER_MARGIN)));
        let inertia = geom.inertia(1.0);
        let center_of_mass = geom.center_of_mass();
        let pos = Isometry3::new(Vector3::new(0.0, 5.0, 0.0), na::zero());
        let handle = world.add_rigid_body(pos, inertia, center_of_mass);
        let box_ch = world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            handle,
            Isometry3::identity(),
            Material::default(),
        );

        let object = box_ch;

        let color = Point3::new(1.0, 0.0, 0.0);

        let shape = geom.as_shape::<Cuboid<f32>>().unwrap();

        let margin = world.collider(object).unwrap().data().margin();
        let rx = shape.half_extents().x + margin;
        let ry = shape.half_extents().y + margin;
        let rz = shape.half_extents().z + margin;
        let delta = na::one();

        let actor = Node::Box(BoxNode::new(
            object, &mut world, delta, rx, ry, rz, color, window,
        ));

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

        AppState {
            arc_ball,
            world,
            ground,
            actor,
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

    fn step(&mut self, _: &mut Window) {
        self.world.step();

        self.ground.update(&self.world);
        self.actor.update(&self.world);
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
