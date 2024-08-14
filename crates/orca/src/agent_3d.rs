use bevy_math::Vec3;

use geometry::colliders::Collider;

#[derive(Clone, Debug)]
pub struct Agent3D {
    pub position: Vec3,
    pub velocity: Vec3,
    pub shape: Collider,
    pub responsibility: f32,
}

impl Agent3D {
    #[must_use]
    pub fn new(position: Vec3, velocity: Vec3, shape: Collider) -> Self {
        Self {
            position,
            velocity,
            shape,
            responsibility: 0.5,
        }
    }
}
