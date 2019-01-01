//! Utilities

pub fn map_range(from_range: (f32, f32), to_range: (f32, f32), s: f32) -> f32 {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}

/// Returns max if self is greater than max, and min if self is less than min.
/// Otherwise this returns self.  Panics if min > max, min equals NaN, or max
/// equals NaN.
///
/// # Examples
///
/// ```
/// assert!((-3.0f32).clamp(-2.0f32, 1.0f32) == -2.0f32);
/// assert!((0.0f32).clamp(-2.0f32, 1.0f32) == 0.0f32);
/// assert!((2.0f32).clamp(-2.0f32, 1.0f32) == 1.0f32);
/// ```
pub fn clamp(val: f32, min: f32, max: f32) -> f32 {
    assert!(min <= max);
    let mut x = val;
    if x < min {
        x = min;
    }
    if x > max {
        x = max;
    }
    x
}
