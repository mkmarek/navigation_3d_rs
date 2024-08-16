use bevy_math::{Mat2, Vec2};

use crate::{Vec2Operations, EPSILON};

#[derive(Debug)]
pub struct Arc2D {
    pub center: Vec2,
    pub start_direction: Vec2,
    pub end_direction: Vec2,
    pub max_angle: f32,
    pub radius: f32,
}

impl Arc2D {
    #[must_use]
    pub fn from_points(radius: f32, start: Vec2, end: Vec2) -> (Self, Self) {
        let midpoint = (start + end) / 2.0;
        let perpendicular_a = (start - midpoint).normalize().perp();
        let perpendicular_b = (end - midpoint).normalize().perp();
        let distance = (start - end).length();
        let perpendicular_component = if (radius - distance / 2.0).abs() < EPSILON {
            0.0
        } else {
            (radius.powi(2) - (distance / 2.0).powi(2)).sqrt()
        };

        let center_a = midpoint + perpendicular_a * perpendicular_component;
        let center_b = midpoint + perpendicular_b * perpendicular_component;

        let arc_a = Self {
            center: center_a,
            start_direction: (start - center_a).normalize(),
            end_direction: (end - center_a).normalize(),
            max_angle: (end - center_a).angle_between(start - center_a),
            radius,
        };

        let arc_b = Self {
            center: center_b,
            start_direction: (start - center_b).normalize(),
            end_direction: (end - center_b).normalize(),
            max_angle: (end - center_b).angle_between(start - center_b),
            radius,
        };

        (arc_a, arc_b)
    }

    pub fn point_at(&self, t: f32) -> Vec2 {
        let direction = {
            let angle = self.start_direction.angle_between(self.end_direction);
            let mut new_angle = angle * t;

            if cross_product(self.start_direction, self.end_direction) < 0.0 {
                new_angle = -new_angle;
            }

            let mat = Mat2::from_cols(
                Vec2::new(new_angle.cos(), new_angle.sin()),
                Vec2::new(-new_angle.sin(), new_angle.cos()),
            );

            mat * self.start_direction
        };

        self.center + direction * self.radius
    }
}

fn cross_product(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

impl Vec2Operations for Arc2D {
    fn contains(&self, pt: Vec2) -> bool {
        let relative = pt - self.center;
        let length_squared = relative.length_squared();

        if (length_squared - self.radius * self.radius).abs() > EPSILON {
            return false;
        }

        let direction = relative.normalize();

        let c1 = cross_product(self.start_direction, direction);
        let c2 = cross_product(direction, self.end_direction);

        c1 > -EPSILON && c2 > -EPSILON
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let relative_pt = pt - self.center;

        let direction = relative_pt.normalize();

        let c1 = cross_product(self.start_direction, direction);
        let c2 = cross_product(direction, self.end_direction);

        if c1 > -EPSILON && c2 > -EPSILON {
            self.center + direction * self.radius
        } else if c1 < 0.0 {
            self.center + self.start_direction * self.radius
        } else {
            self.center + self.end_direction * self.radius
        }
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let constrainted = self.constrain(pt);
        let normal = (constrainted - self.center).normalize_or_zero();

        (constrainted, normal)
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        let constrainted = self.constrain(pt);

        (constrainted - pt).length()
    }
}
