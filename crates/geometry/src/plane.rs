use bevy_math::{Vec2, Vec3};

use crate::{Hyperplane, Ray2DIntersection, Vec2Operations, Vec3Operations, EPSILON};

#[derive(Debug, Clone)]
pub struct Plane {
    pub normal: Vec3,
    pub origin: Vec3,
    pub u_direction: Vec3,
    pub v_direction: Vec3,
}

impl Plane {
    #[must_use]
    pub fn from_points(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let normal = (b - a).cross(c - a).normalize();
        let origin = a;

        Self::new(origin, normal)
    }

    #[must_use]
    pub fn from_hyperplane_intersection(plane: &Hyperplane, other: &Hyperplane) -> Option<Self> {
        let normal_x = plane.u_direction.dot(other.normal);
        let normal_y = plane.v_direction.dot(other.normal);
        let normal_z = plane.w_direction.dot(other.normal);

        let d2 = other.origin.dot(other.normal);
        let d_result = d2 - plane.origin.dot(other.normal);

        if normal_x.abs() < EPSILON && normal_y.abs() < EPSILON && normal_z.abs() < EPSILON {
            return None;
        }

        let normal = Vec3::new(normal_x, normal_y, normal_z).normalize();
        let origin = d_result * normal;

        Some(Self::new(origin, normal))
    }

    #[must_use]
    pub fn new(origin: Vec3, normal: Vec3) -> Self {
        let normal = normal.normalize();

        let basis = {
            let x_dot = Vec3::X.dot(normal).abs();
            let y_dot = Vec3::Y.dot(normal).abs();
            let z_dot = Vec3::Z.dot(normal).abs();

            if x_dot < y_dot && x_dot < z_dot {
                Vec3::X
            } else if y_dot < z_dot {
                Vec3::Y
            } else {
                Vec3::Z
            }
        };

        let u_direction = normal.cross(basis).normalize();
        let v_direction = normal.cross(u_direction).normalize();

        Self {
            normal,
            origin,
            u_direction,
            v_direction,
        }
    }

    #[must_use]
    pub fn project_2d(&self, p: Vec3) -> Vec2 {
        let u = self.u_direction.dot(p - self.origin);
        let v = self.v_direction.dot(p - self.origin);

        Vec2::new(u, v)
    }

    #[must_use]
    pub fn project_3d(&self, p: Vec2) -> Vec3 {
        Vec3::new(
            self.origin.x + p.x * self.u_direction.x + p.y * self.v_direction.x,
            self.origin.y + p.x * self.u_direction.y + p.y * self.v_direction.y,
            self.origin.z + p.x * self.u_direction.z + p.y * self.v_direction.z,
        )
    }
}

pub trait PlaneIntersecionShape: Vec2Operations + Ray2DIntersection {}
impl<T> PlaneIntersecionShape for T where T: Vec2Operations + Ray2DIntersection {}

pub trait PlaneIntersecion {
    fn intersect(&self, plane: &Plane) -> Option<impl PlaneIntersecionShape>;
}

impl Vec3Operations for Plane {
    fn contains(&self, pt: Vec3) -> bool {
        self.signed_distance(pt) >= -EPSILON
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let t = self.normal.dot(self.origin) - self.normal.dot(pt);

        pt + self.normal * t
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let t = self.normal.dot(self.origin) - self.normal.dot(pt);
        let closest_point = pt + self.normal * t;

        (closest_point, self.normal)
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        self.normal.dot(pt - self.origin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane_new() {
        let origin = Vec3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let plane = Plane::new(origin, normal);

        assert_eq!(plane.origin, origin);
        assert_eq!(plane.normal, normal);

        // u and v vectors should be orthogonal to the normal
        assert!(plane.u_direction.dot(plane.normal).abs() < f32::EPSILON);
        assert!(plane.v_direction.dot(plane.normal).abs() < f32::EPSILON);

        // u and v vectors should be orthogonal to each other
        assert!(plane.u_direction.dot(plane.v_direction).abs() < f32::EPSILON);
    }

    #[test]
    fn test_plane_from_points() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let plane = Plane::from_points(a, b, c);

        assert_eq!(plane.origin, a);
        assert_eq!(plane.normal, Vec3::new(0.0, 0.0, 1.0));

        // u and v vectors should be orthogonal to the normal
        assert!(plane.u_direction.dot(plane.normal).abs() < f32::EPSILON);
        assert!(plane.v_direction.dot(plane.normal).abs() < f32::EPSILON);

        // u and v vectors should be orthogonal to each other
        assert!(plane.u_direction.dot(plane.v_direction).abs() < f32::EPSILON);
    }

    #[test]
    fn test_plane_project_2d() {
        let origin = Vec3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let plane = Plane::new(origin, normal);
        let point = Vec3::new(1.0, 0.0, 1.0);

        assert_eq!(plane.project_2d(point), Vec2::new(1.0, -1.0));
    }

    #[test]
    fn test_plane_project_3d() {
        let origin = Vec3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let plane = Plane::new(origin, normal);
        let point = Vec2::new(1.0, 1.0);

        assert_eq!(plane.project_3d(point), Vec3::new(1.0, 0.0, -1.0));
    }

    #[test]
    fn test_plane_contains_point() {
        let origin = Vec3::new(0.0, 0.0, 0.0);
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let plane = Plane::new(origin, normal);
        let point = Vec3::new(0.0, 1.0, 0.0);

        assert!(plane.contains(point));
    }
}
