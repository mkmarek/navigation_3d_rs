use bevy_math::Vec2;

use crate::{Plane, Vec2Operations, EPSILON};

#[derive(Clone, Debug)]
pub struct HalfPlane {
    pub normal: Vec2,
    pub point: Vec2,
}

impl HalfPlane {
    #[must_use]
    pub fn new(point: Vec2, normal: Vec2) -> Self {
        Self {
            normal: normal.normalize(),
            point,
        }
    }

    #[must_use]
    // https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
    pub fn from_plane_intersection(plane: &Plane, other: &Plane) -> Option<Self> {
        let d1 = plane.normal.dot(plane.origin);
        let d2 = other.normal.dot(other.origin);

        let normals_dot = plane.normal.dot(other.normal);
        let denominator = 1.0 - normals_dot.powi(2);

        if (denominator).abs() < EPSILON {
            return None;
        }

        let k1 = (d1 - d2 * normals_dot) / denominator;
        let k2 = (d2 - d1 * normals_dot) / denominator;

        let pt = plane.normal * k1 + other.normal * k2;
        let cross = plane.normal.cross(other.normal);
        let pt2 = pt + cross;

        let plane_pt = plane.project_2d(pt);
        let plane_pt2 = plane.project_2d(pt2);

        let result = Self::new(plane_pt, (plane_pt - plane_pt2).normalize().perp());
        Some(result)
    }
}

impl Vec2Operations for HalfPlane {
    fn contains(&self, pt: Vec2) -> bool {
        self.signed_distance(pt) >= -EPSILON
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        let signed_distance = self.signed_distance(pt);
        if signed_distance >= -EPSILON {
            pt
        } else {
            pt - self.normal * signed_distance
        }
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        let signed_distance = self.signed_distance(pt);
        if signed_distance >= -EPSILON {
            (pt, self.normal)
        } else {
            (pt - self.normal * signed_distance, self.normal)
        }
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        self.normal.dot(pt - self.point)
    }
}

#[cfg(test)]
mod tests {}
