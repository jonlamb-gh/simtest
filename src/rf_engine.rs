use crate::box_node::update_scene_node;
use crate::na::{Point3, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nphysics3d::algebra::ForceType;
use nphysics3d::math::Force;
use nphysics3d::object::{
    BodyPartHandle, DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet,
};

pub struct RfEngine {
    body_part: BodyPartHandle<DefaultBodyHandle>,
    collider: DefaultColliderHandle,
    node: SceneNode,
    force: Force<f32>,
}

impl RfEngine {
    pub fn new(
        body_part: BodyPartHandle<DefaultBodyHandle>,
        collider: DefaultColliderHandle,
        node: SceneNode,
    ) -> Self {
        RfEngine {
            body_part,
            collider,
            node,
            force: Force::zero(),
        }
    }

    pub fn update(&mut self, colliders: &DefaultColliderSet<f32>) {
        update_scene_node(self.collider, colliders, &mut self.node);
    }

    pub fn size() -> Vector3<f32> {
        Vector3::new(0.1, 0.14, 0.1)
    }

    pub fn force(&self) -> &Force<f32> {
        &self.force
    }

    pub fn set_force(
        &mut self,
        local_force: Force<f32>,
        bodies: &mut DefaultBodySet<f32>,
        colliders: &DefaultColliderSet<f32>,
    ) {
        self.force = local_force;

        let co = colliders.get(self.collider).unwrap();
        let p = co.position_wrt_body();
        //let point = Point3::new(p.translation.x, p.translation.y, p.translation.z);
        let point = Point3::new(1.0, p.translation.y, 1.0);
        //let body = bodies.get_mut(self.body_part.0).unwrap();
        let body = bodies.get_mut(co.body()).unwrap();

        //println!("{:#?}", p);
        //println!("{:#?}", point);

        body.apply_local_force_at_local_point(
            //self.body_part.1,
            0,
            &self.force.linear,
            &point,
            ForceType::Force,
            true,
        );

        //body.apply_local_force(0, &self.force, ForceType::Force, true);
        //body.apply_local_force(self.body_part.1, &self.force, ForceType::Force, true);
    }

    pub fn draw_force_vector(&self, colliders: &DefaultColliderSet<f32>, win: &mut Window) {
        // TODO - configs
        let color = Point3::new(0.0, 0.0, 1.0);
        let scale = 0.25;

        let p = colliders.get(self.collider).unwrap().position();
        let f = p.rotation.transform_vector(&self.force().linear);
        let a = p.translation.vector;
        let b = a + (f * scale);

        win.draw_line(&Point3::from(a), &Point3::from(b), &color);
    }
}
