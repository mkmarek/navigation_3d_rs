use bevy_math::Vec3;

use crate::{Vec3Operations, EPSILON};

pub struct Ray3D {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray3D {
    #[must_use]
    pub fn new(origin: Vec3, direction: Vec3) -> Self {
        Self {
            origin,
            direction: direction.normalize(),
        }
    }

    #[must_use]
    pub fn at(&self, t: f32) -> Vec3 {
        self.origin + self.direction * t
    }

    /// Returns the parameter `t` at which the given point is located from the origin along the ray.
    ///
    /// # Arguments
    ///
    /// * `pt` - A Vec3 that represents the point.
    ///
    /// # Returns
    ///
    /// * A float that represents the parameter `t`.
    pub fn parameter_at_point(&self, pt: Vec3) -> f32 {
        let relative_pt = pt - self.origin;

        relative_pt.dot(self.direction)
    }
}

impl Vec3Operations for Ray3D {
    fn contains(&self, pt: Vec3) -> bool {
        let projected = (pt - self.origin).dot(self.direction) * self.direction;

        (projected - self.origin).length() < EPSILON
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        (pt - self.origin).dot(self.direction) * self.direction
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let projected = (pt - self.origin).dot(self.direction) * self.direction;
        let normal = (projected - pt).normalize_or_zero();

        (projected, normal)
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let projected = (pt - self.origin).dot(self.direction) * self.direction;

        (projected - pt).length()
    }
}
