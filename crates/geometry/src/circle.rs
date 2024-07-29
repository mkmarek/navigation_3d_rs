use bevy_math::Vec2;

use crate::{line_segment_2d::LineSegment2D, points::Vec2Operations, ray_2d::*, EPSILON};

pub struct Circle {
    pub radius: f32,
    pub origin: Vec2,
}

impl Circle {
    #[must_use]
    pub fn new(radius: f32, origin: Vec2) -> Self {
        Self { radius, origin }
    }
}

impl Ray2DIntersection for Circle {
    fn intersect(&self, ray: &Ray2D) -> Ray2DIntersectionResult {
        let point = ray.origin - self.origin;
        let direction = ray.direction;
        let radius = self.radius;

        let dr = direction.length();

        let x1 = point.x;
        let y1 = point.y;
        let x2 = point.x + direction.x;
        let y2 = point.y + direction.y;

        let d = x1 * y2 - x2 * y1;
        let discriminant = radius * radius * dr * dr - d * d;

        if discriminant < 0.0 {
            return Ray2DIntersectionResult::None;
        }

        if discriminant < EPSILON {
            let t = if direction.x.abs() < direction.y.abs() {
                let y = -d * direction.x / (dr * dr);
                (y - y1) / direction.y
            } else {
                let x = d * direction.y / (dr * dr);
                (x - x1) / direction.x
            };

            Ray2DIntersectionResult::Point(t)
        } else {
            let ta = if direction.x.abs() < direction.y.abs() {
                let ya = -d * direction.x + direction.y.abs() * discriminant.sqrt() / (dr * dr);
                (ya - y1) / direction.y
            } else {
                let xa = (d * direction.y
                    + direction.y.signum() * direction.x * discriminant.sqrt())
                    / (dr * dr);
                (xa - x1) / direction.x
            };

            let tb = if direction.x.abs() < direction.y.abs() {
                let yb = -d * direction.x - direction.y.abs() * discriminant.sqrt() / (dr * dr);
                (yb - y1) / direction.y
            } else {
                let xb = (d * direction.y
                    - direction.y.signum() * direction.x * discriminant.sqrt())
                    / (dr * dr);
                (xb - x1) / direction.x
            };

            let t_min = ta.min(tb);
            let t_max = ta.max(tb);

            Ray2DIntersectionResult::LineSegment(LineSegment2D::new(
                ray.origin,
                ray.direction,
                t_min,
                t_max,
            ))
        }
    }
}

impl Vec2Operations for Circle {
    fn contains(&self, pt: Vec2) -> bool {
        let relative_pt = pt - self.origin;

        relative_pt.length_squared() <= self.radius * self.radius
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let relative_pt = pt - self.origin;

        if relative_pt.length_squared() > self.radius * self.radius {
            self.origin + relative_pt.normalize() * self.radius
        } else {
            pt
        }
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let relative_pt = pt - self.origin;
        let normal = relative_pt.normalize();
        let closest_point = self.origin + normal * self.radius;

        (closest_point, normal)
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        let relative_pt = pt - self.origin;

        relative_pt.length() - self.radius
    }
}
