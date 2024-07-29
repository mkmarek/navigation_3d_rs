use bevy_math::{Mat3, Vec2};

use crate::{
    line_segment_2d::LineSegment2D, Ray2D, Ray2DIntersection, Ray2DIntersectionResult,
    Vec2Operations, EPSILON,
};

// This shape is a result of intersecting spherinder with a hyperplane and then intersecting the resulting shape with a plane.
#[derive(Debug)]
pub struct SpherinderHyperplanePlaneIntersection {
    semi_major_axis: f32,
    semi_minor_axis: f32,
    #[allow(dead_code)]
    center: Vec2,
    transform: Mat3,
    transform_inv: Mat3,
}

impl SpherinderHyperplanePlaneIntersection {
    #[allow(clippy::many_single_char_names)]
    #[must_use]
    pub fn new(semi_major_axis: f32, semi_minor_axis: f32, center: Vec2, transform: Mat3) -> Self {
        Self {
            semi_major_axis,
            semi_minor_axis,
            center,
            transform,
            transform_inv: transform.inverse(),
        }
    }
}

impl Vec2Operations for SpherinderHyperplanePlaneIntersection {
    fn contains(&self, _pt: Vec2) -> bool {
        todo!()
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let pt_ellipse_space = self.transform_inv.transform_point2(pt);

        // Ellipse equation: x^2/a^2 + y^2/b^2 = 1
        // Check if the point is inside the ellipse

        let a_sq = self.semi_major_axis.powi(2);
        let b_sq = self.semi_minor_axis.powi(2);

        let x = pt_ellipse_space.x;
        let y = pt_ellipse_space.y;

        let x_sq = x.powi(2);
        let y_sq = y.powi(2);

        if x_sq / a_sq + y_sq / b_sq <= 1.0 {
            pt
        } else {
            // If the point is outside then normalize it
            let angle = y.atan2(x);
            let x = self.semi_major_axis * angle.cos();
            let y = self.semi_minor_axis * angle.sin();

            self.transform.transform_point2(Vec2::new(x, y))
        }
    }

    fn closest_point_and_normal(&self, _pt: Vec2) -> (Vec2, Vec2) {
        todo!()
    }

    fn signed_distance(&self, _pt: Vec2) -> f32 {
        todo!()
    }
}

impl Ray2DIntersection for SpherinderHyperplanePlaneIntersection {
    fn intersect(&self, ray: &Ray2D) -> Ray2DIntersectionResult {
        let origin = ray.origin;
        let direction = ray.direction;
        let origin_ellipse_space = self.transform_inv.transform_point2(origin);
        let direction_ellipse_space = self.transform_inv.transform_vector2(direction);

        let a = self.semi_major_axis;
        let b = self.semi_minor_axis;
        let a_sq = a.powi(2);
        let b_sq = b.powi(2);
        let x0 = origin_ellipse_space.x;
        let y0 = origin_ellipse_space.y;
        let dx = direction_ellipse_space.x;
        let dy = direction_ellipse_space.y;

        let dx_sq = dx.powi(2);
        let dy_sq = dy.powi(2);
        let x0_sq = x0.powi(2);
        let y0_sq = y0.powi(2);

        let k =
            b_sq * dx_sq + a_sq * dy_sq - dy_sq * x0_sq + 2.0 * dx * dy * x0 * y0 - dx_sq * y0_sq;

        if k < 0.0 {
            return Ray2DIntersectionResult::None;
        }

        let denominator = b_sq * dx_sq + a_sq * dy_sq;

        if denominator.abs() < EPSILON {
            return Ray2DIntersectionResult::None;
        }

        let h = b_sq * dx * x0 + a_sq * dy * y0;

        if k.abs() < EPSILON {
            let t = -h / denominator;

            Ray2DIntersectionResult::Point(t)
        } else {
            let t1 = (-h + k.sqrt() * a * b) / denominator;
            let t2 = (-h - k.sqrt() * a * b) / denominator;

            let t_min = t2.min(t1);
            let t_max = t2.max(t1);

            Ray2DIntersectionResult::LineSegment(LineSegment2D {
                origin,
                direction,
                t_min,
                t_max,
            })
        }
    }
}
