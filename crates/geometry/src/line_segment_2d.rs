use bevy_math::Vec2;

use crate::{Ray2D, Ray2DIntersection, Ray2DIntersectionResult, Vec2Operations, EPSILON};

#[derive(Debug)]
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

    #[must_use]
    pub fn from_two_points(p1: Vec2, p2: Vec2) -> Self {
        let direction = p2 - p1;
        let t_min = 0.0;
        let t_max = direction.length();

        let direction = direction / t_max;

        Self::new(p1, direction, t_min, t_max)
    }

    pub fn to_ray(&self) -> Ray2D {
        Ray2D::new(self.origin, self.direction)
    }

    pub fn end(&self) -> Vec2 {
        self.origin + self.direction * self.t_max
    }
}

impl Vec2Operations for LineSegment2D {
    fn contains(&self, pt: Vec2) -> bool {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        if t < self.t_min || t > self.t_max {
            return false;
        }

        let projected_pt = t * self.direction;

        (projected_pt - relative_pt).length_squared() < EPSILON
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let relative_pt = pt - self.origin;
        let t = relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max);

        self.origin + self.direction * t
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let relative_pt = pt - self.origin;
        let t = relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max);

        let projected_pt = t * self.direction;

        let mut normal = self.direction.perp();

        if (relative_pt - projected_pt).dot(normal) < 0.0 {
            normal = -normal;
        }

        (self.origin + projected_pt, normal)
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        let relative_pt = pt - self.origin;
        let t = relative_pt
            .dot(self.direction)
            .clamp(self.t_min, self.t_max);

        (self.origin + self.direction * t - pt).length()
    }
}

// Represents the result of a 2D ray intersection.
// None: No intersection.
// One: One intersection at the parameter t.
// Two: Two intersections at the parameters t1 and t2.
//
// The parameters t, t1 and t2 represent the distance from the ray origin to the intersection point, they scale with the norm of the direction vector.
// The intersection point is calculated as origin + direction * t.
pub enum LineSegment2DIntersectionResult {
    None,
    Point(f32),
    LineSegment(LineSegment2D),
}

// Represents an object that can be intersected by a 2D ray.
pub trait LineSegment2DIntersection {
    fn intersect(&self, line: &LineSegment2D) -> LineSegment2DIntersectionResult;
}

impl LineSegment2DIntersection for LineSegment2D {
    fn intersect(&self, line: &LineSegment2D) -> LineSegment2DIntersectionResult {
        let ray_1 = self.to_ray();
        let ray_2 = line.to_ray();

        let result = ray_1.intersect(&ray_2);

        match result {
            Ray2DIntersectionResult::None => LineSegment2DIntersectionResult::None,
            Ray2DIntersectionResult::Point(t) => {
                if t < self.t_min || t > self.t_max {
                    LineSegment2DIntersectionResult::None
                } else {
                    LineSegment2DIntersectionResult::Point(t)
                }
            }
            Ray2DIntersectionResult::LineSegment(line) => {
                let t_min = line.t_min.clamp(self.t_min, self.t_max);
                let t_max = line.t_max.clamp(self.t_min, self.t_max);

                if t_min > t_max || (t_max - t_min).abs() < EPSILON {
                    LineSegment2DIntersectionResult::None
                } else {
                    LineSegment2DIntersectionResult::LineSegment(LineSegment2D::new(
                        self.origin,
                        self.direction,
                        t_min,
                        t_max,
                    ))
                }
            }
        }
    }
}
