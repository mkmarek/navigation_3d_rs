use bevy_math::Vec3;

use crate::{Vec3Operations, EPSILON};

pub struct InfiniteCone {
    pub vertex: Vec3,
    pub direction: Vec3,
    pub radius: f32,
}

impl InfiniteCone {
    // Creates a new infinite cone from a vertex, direction, and radius.
    // The radius determines the radius of the base of the cone if the base was at distance equal
    // to the length of the direction vector.
    #[must_use]
    pub fn new(vertex: Vec3, direction: Vec3, radius: f32) -> Self {
        let radius = radius / direction.length();
        Self {
            vertex,
            direction: direction.normalize(),
            radius,
        }
    }
}

impl Vec3Operations for InfiniteCone {
    fn contains(&self, pt: Vec3) -> bool {
        let relative_pt = pt - self.vertex;
        let height = relative_pt.dot(self.direction);

        if height < 0.0 {
            return false;
        }

        let projection = self.direction * height;
        let perpendicular_component_sq = (relative_pt - projection).length_squared();

        if perpendicular_component_sq > self.radius * self.radius * height * height {
            return false;
        }

        true
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        if self.contains(pt) {
            return pt;
        }

        let (closest_point, _) = self.closest_point_and_normal(pt);

        closest_point
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let relative_pt = pt - self.vertex;
        let height = relative_pt.dot(self.direction).max(0.0);
        let pt_projected = self.direction * height;

        let perpendicular_component = (relative_pt - pt_projected).length_squared();
        let perpendicular_component = if perpendicular_component < EPSILON {
            0.0
        } else {
            perpendicular_component.sqrt()
        };

        let perpendicular_direction = if perpendicular_component.abs() < EPSILON {
            let dot_x = Vec3::X.dot(self.direction);
            let dot_y = Vec3::Y.dot(self.direction);
            let dot_z = Vec3::Z.dot(self.direction);

            let basis_angle = if dot_x.abs() < dot_y.abs() && dot_x.abs() < dot_z.abs() {
                Vec3::X
            } else if dot_y.abs() < dot_z.abs() {
                Vec3::Y
            } else {
                Vec3::Z
            };

            self.direction.cross(basis_angle).normalize()
        } else {
            (relative_pt - pt_projected) / perpendicular_component
        };

        let b = perpendicular_component + 1.0 / self.radius * height;

        let x = b * self.radius / (self.radius * self.radius + 1.0);
        let y = self.radius * x;

        let closest_point = self.vertex + self.direction * x + perpendicular_direction * y;

        let normal = if perpendicular_component < self.radius * height {
            (closest_point - pt).normalize()
        } else {
            (pt - closest_point).normalize()
        };

        (closest_point, normal)
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let (closest_point, normal) = self.closest_point_and_normal(pt);

        if self.contains(pt) {
            (pt - closest_point).dot(normal)
        } else {
            (closest_point - pt).length()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_math::Vec3;

    #[test]
    fn test_new() {
        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
        assert_eq!(cone.vertex, Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(cone.direction, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(cone.radius, 1.0);
    }

    #[test]
    fn test_contains() {
        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
        assert!(cone.contains(Vec3::new(0.5, 0.0, 0.0)));
        assert!(!cone.contains(Vec3::new(2.0, 10.0, 0.0)));
    }

    #[test]
    fn test_constrain() {
        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
        assert_eq!(
            cone.constrain(Vec3::new(0.5, 0.0, 0.0)),
            Vec3::new(0.5, 0.0, 0.0)
        );
        assert_eq!(
            cone.constrain(Vec3::new(2.0, 10.0, 0.0)),
            Vec3::new(6.0, 6.0, 0.0)
        );
    }

    #[test]
    fn test_closest_point_and_normal() {
        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
        let (closest_point, normal) = cone.closest_point_and_normal(Vec3::new(0.5, 0.0, 0.0));
        assert_eq!(closest_point, Vec3::new(0.25, -0.25, 0.0));
        assert_eq!(normal, Vec3::new(-0.70710677, -0.70710677, 0.0));

        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), 1.0);
        let (closest_point, normal) = cone.closest_point_and_normal(Vec3::new(-0.5, 0.0, 0.0));
        assert_eq!(closest_point, Vec3::new(-0.25, 0.25, 0.0));
        assert_eq!(normal, Vec3::new(0.70710677, 0.70710677, 0.0));
    }

    #[test]
    fn test_signed_distance() {
        let cone = InfiniteCone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
        assert_eq!(cone.signed_distance(Vec3::new(0.5, 0.0, 0.0)), -0.35355338);
    }
}
