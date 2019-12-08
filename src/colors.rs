use crate::na::Point3;
use random_color::RandomColor;

pub struct Colors {}

impl Colors {
    pub fn new() -> Point3<f32> {
        let color = RandomColor::new().to_rgb_array();
        Point3::new(
            color[0] as f32 / 255.0,
            color[1] as f32 / 255.0,
            color[2] as f32 / 255.0,
        )
    }
}
