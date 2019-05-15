use crate::na::{Isometry3, Vector3};
use nphysics3d::math::Velocity;
use nphysics3d::object::ColliderDesc;
use nphysics3d::world::World;

pub struct PartDesc {
    pub size: Vector3<f32>,
    //pub shape: Cuboid<f32>,
    pub mass: f32,
    pub density: f32,
}

pub trait Part {
    fn part_desc() -> PartDesc;

    fn collider_desc() -> ColliderDesc<f32>;

    fn position(&self, world: &World<f32>) -> Isometry3<f32>;

    fn velocity(&self, world: &World<f32>) -> Velocity<f32>;
}
