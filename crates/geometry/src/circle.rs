use bevy_math::Vec2;

use crate::{line_segment_2d::LineSegment2D, points::Vec2Operations, ray_2d::*, EPSILON};

pub struct Circle {
    pub radius: f32,
    pub origin: Vec2,
}

#[derive(Debug)]
pub enum SecondTangentPointResult {
    None,
    Point(f32),
    TwoPoints(f32, f32),
}

impl Circle {
    #[must_use]
    pub fn new(radius: f32, origin: Vec2) -> Self {
        Self { radius, origin }
    }

    // Finds the second tangent of the circle from the first tangent and a desired direction
    // of the second tangent.
    // Returns parameter t, which is the distance from the origin of the first tangent to the
    // origin of the second tangent.
    // There can be always two tangents that lie on the given ray and are tangent to the circle
    // with the given direction
    pub fn find_tangent_on_ray(
        &self,
        first_tangent: &Ray2D,
        direction: Vec2,
    ) -> SecondTangentPointResult {
        let first_tangent_length = first_tangent.direction.length();
        let first_tangent_direction = first_tangent.direction.normalize();
        let direction = direction.normalize();

        if first_tangent_direction.dot(direction).abs() >= 0.9 {
            return SecondTangentPointResult::None;
        }

        let denominator =
            direction.x * first_tangent_direction.y - direction.y * first_tangent_direction.x;

        if denominator.abs() < EPSILON {
            return SecondTangentPointResult::None;
        }

        let numerator = direction.x * (first_tangent.origin.y - self.origin.y)
            - direction.y * (first_tangent.origin.x - self.origin.x);

        let t1 = -(numerator + self.radius) / denominator * first_tangent_length;
        let t2 = -(numerator - self.radius) / denominator * first_tangent_length;

        SecondTangentPointResult::TwoPoints(t1, t2)
    }

    // Finds the second tangent of the circle from the first tangent and a desired direction
    // of the second tangent.
    // Returns parameter t, which is the distance from the origin of the first tangent to the
    // origin of the second tangent.
    // There can be always two tangents that lie on the given line segment and are tangent to the circle
    // with the given direction
    pub fn find_tangent_on_line_segment(
        &self,
        first_tangent: &LineSegment2D,
        direction: Vec2,
    ) -> SecondTangentPointResult {
        match self.find_tangent_on_ray(&first_tangent.to_ray(), direction) {
            SecondTangentPointResult::None => SecondTangentPointResult::None,
            SecondTangentPointResult::Point(t) => {
                if first_tangent.t_min <= t && t <= first_tangent.t_max {
                    SecondTangentPointResult::Point(t)
                } else {
                    SecondTangentPointResult::None
                }
            }
            SecondTangentPointResult::TwoPoints(t1, t2) => {
                if first_tangent.t_min <= t1 && t1 <= first_tangent.t_max {
                    if first_tangent.t_min <= t2 && t2 <= first_tangent.t_max {
                        SecondTangentPointResult::TwoPoints(t1, t2)
                    } else {
                        SecondTangentPointResult::Point(t1)
                    }
                } else if first_tangent.t_min <= t2 && t2 <= first_tangent.t_max {
                    SecondTangentPointResult::Point(t2)
                } else {
                    SecondTangentPointResult::None
                }
            }
        }
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

    //pub fn find_second_tangent(&self, ray: &Ray2D) -> Option<
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
