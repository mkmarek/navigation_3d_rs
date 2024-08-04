use bevy_math::Vec2;

use crate::{Ray2D, Vec2Operations, EPSILON};

pub struct LineSegment2D {
    pub origin: Vec2,
    pub direction: Vec2,
    pub t_min: f32,
    pub t_max: f32,
}

impl LineSegment2D {
    // Represents a 2D line segment defined by origin, direction and its bounds
    #[must_use]
    pub fn new(origin: Vec2, direction: Vec2, t_min: f32, t_max: f32) -> Self {
        Self {
            origin,
            direction,
            t_min,
            t_max,
        }
    }

    pub fn to_ray(&self) -> Ray2D {
        Ray2D::new(self.origin, self.direction)
    }
}

impl Vec2Operations for LineSegment2D {
    fn contains(&self, pt: Vec2) -> bool {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        t >= (self.t_min - EPSILON) && t <= (self.t_max + EPSILON)
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        if t < self.t_min {
            return self.origin + self.direction * self.t_min;
        }

        if t > self.t_max {
            return self.origin + self.direction * self.t_max;
        }

        pt
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        if t < self.t_min {
            return (self.origin + self.direction * self.t_min, -self.direction);
        }

        if t > self.t_max {
            return (self.origin + self.direction * self.t_max, self.direction);
        }

        (self.origin + self.direction * t, self.direction)
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        if t < self.t_min {
            return (self.origin + self.direction * self.t_min - pt).length();
        }

        if t > self.t_max {
            return (self.origin + self.direction * self.t_max - pt).length();
        }

        (self.origin + self.direction * t - pt).length()
    }
}
