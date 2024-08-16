use bevy_math::Vec2;

use crate::{line_segment_2d::LineSegment2D, Vec2Operations, EPSILON};

pub struct Ray2D {
    pub origin: Vec2,
    pub direction: Vec2,
}

impl Ray2D {
    #[must_use]
    pub fn new(origin: Vec2, direction: Vec2) -> Self {
        Self { origin, direction }
    }
}

// Represents the result of a 2D ray intersection.
// None: No intersection.
// One: One intersection at the parameter t.
// Two: Two intersections at the parameters t1 and t2.
//
// The parameters t, t1 and t2 represent the distance from the ray origin to the intersection point, they scale with the norm of the direction vector.
// The intersection point is calculated as origin + direction * t.
pub enum Ray2DIntersectionResult {
    None,
    Point(f32),
    LineSegment(LineSegment2D),
}

// Represents an object that can be intersected by a 2D ray.
pub trait Ray2DIntersection {
    fn intersect(&self, ray: &Ray2D) -> Ray2DIntersectionResult;
}

impl Ray2DIntersection for Ray2D {
    fn intersect(&self, ray: &Ray2D) -> Ray2DIntersectionResult {
        let point = self.origin;
        let direction = self.direction;

        let other_point = ray.origin;
        let other_direction = ray.direction;

        // Check if the lines are parallel
        if (direction.x * other_direction.y - direction.y * other_direction.x).abs() <= EPSILON {
            return Ray2DIntersectionResult::None;
        }

        // Before calculating slope we need to check if the direction vector is vertical
        if other_direction.x.abs() <= EPSILON {
            // if the line is horizontal the whole line lies on the point y = other_point.y
            // therefore the parameter t will be:
            Ray2DIntersectionResult::Point((other_point.x - point.x) / direction.x)
        } else if other_direction.y.abs() <= EPSILON {
            // if the line is vertical the whole line lies on the point x = other_point.x
            // therefore the parameter t will be:
            Ray2DIntersectionResult::Point((other_point.y - point.y) / direction.y)
        } else if other_direction.x.abs() > other_direction.y.abs() {
            // Calculate the slope of the other_line
            let slope = other_direction.y / other_direction.x;

            // Calculate the y-intercept of the other_line
            let y_intercept = other_point.y - slope * other_point.x;

            // now we have the line defined as: y = slope * x + y_intercept
            // assigning the parametric equations of the first line to the second line
            // we get: (y_0 + t * dy) = slope * (x_0 + t * dx) + y_intercept
            // expressed in terms of t we get: t= -(slope*x_0 - y_0 + y_intercept)/(dx*slope - dy)

            let denominator = direction.x * slope - direction.y;

            if denominator.abs() <= EPSILON {
                return Ray2DIntersectionResult::None;
            }

            Ray2DIntersectionResult::Point(-(slope * point.x - point.y + y_intercept) / denominator)
        } else {
            // Calculate the slope of the other_line
            let slope = other_direction.x / other_direction.y;

            // Calculate the x-intercept of the other_line
            let x_intercept = other_point.x - slope * other_point.y;

            // now we have the line defined as: x = slope * y + x_intercept
            // assigning the parametric equations of the first line to the second line
            // we get: (x_0 + t * dx) = slope * (y_0 + t * dy) + x_intercept
            // expressed in terms of t we get: t= -(slope*y_0 - x_0 + x_intercept)/(dy*slope - dx)

            let denominator = direction.y * slope - direction.x;

            if denominator.abs() <= EPSILON {
                return Ray2DIntersectionResult::None;
            }

            Ray2DIntersectionResult::Point(-(slope * point.y - point.x + x_intercept) / denominator)
        }
    }
}

impl Vec2Operations for Ray2D {
    fn contains(&self, pt: Vec2) -> bool {
        let relative_pt = pt - self.origin;
        let projected_pt = relative_pt.dot(self.direction) * self.direction;

        (projected_pt - relative_pt).length_squared() < EPSILON
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        self.origin + self.direction * t
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let relative_pt = pt - self.origin;
        let projected_pt = relative_pt.dot(self.direction) * self.direction;

        let mut normal = self.direction.perp();

        if (relative_pt - projected_pt).dot(normal) < 0.0 {
            normal = -normal;
        }

        (self.origin + projected_pt, normal)
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        let relative_pt = pt - self.origin;
        let t = relative_pt.dot(self.direction);

        (self.origin + self.direction * t - pt).length()
    }
}
