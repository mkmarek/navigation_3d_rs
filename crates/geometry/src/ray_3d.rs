use bevy_math::Vec3;

pub struct Ray3D {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray3D {
    #[must_use]
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self { origin, direction }
    }
}
