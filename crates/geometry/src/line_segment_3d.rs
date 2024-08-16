use bevy_math::Vec3;

use crate::{Ray3D, Vec3Operations, EPSILON};

pub struct LineSegment3D {
    pub origin: Vec3,
    pub direction: Vec3,
    pub t_min: f32,
    pub t_max: f32,
}

impl LineSegment3D {
    // Represents a 3D line segment defined by origin, direction and its bounds
    #[must_use]
    pub fn new(origin: Vec3, direction: Vec3, t_min: f32, t_max: f32) -> Self {
        Self {
            origin,
            direction,
            t_min,
            t_max,
        }
    }

    #[must_use]
    pub fn from_two_points(p1: Vec3, p2: Vec3) -> Self {
        let direction = p2 - p1;
        let t_min = 0.0;
        let t_max = direction.length();

        let direction = direction / t_max;

        Self::new(p1, direction, t_min, t_max)
    }

    pub fn to_ray(&self) -> Ray3D {
        Ray3D::new(self.origin, self.direction)
    }

    pub fn length(&self) -> f32 {
        self.t_max - self.t_min
    }

    pub fn parameter_at_point(&self, pt: Vec3) -> f32 {
        let relative_pt = pt - self.origin;

        relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max)
            / self.length()
    }
}

impl Vec3Operations for LineSegment3D {
    fn contains(&self, pt: Vec3) -> bool {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        if t < self.t_min || t > self.t_max {
            return false;
        }

        let projected_pt = t * self.direction;

        (projected_pt - relative_pt).length_squared() < EPSILON
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let relative_pt = pt - self.origin;
        let t = relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max);

        self.origin + self.direction * t
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);
        let projected_pt = t * self.direction;

        let normal = (relative_pt - projected_pt).normalize_or_zero();

        let t = t.clamp(self.t_min, self.t_max);
        let projected_pt = t * self.direction;

        (self.origin + projected_pt, normal)
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let relative_pt = pt - self.origin;
        let t = relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max);

        (self.origin + self.direction * t - pt).length()
    }
}
