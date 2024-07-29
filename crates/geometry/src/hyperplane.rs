use bevy_math::{Vec3, Vec4};

use crate::{PlaneIntersecion, Vec3Operations, Vec4Operations, EPSILON};

#[derive(Debug, Clone)]
pub struct Hyperplane {
    pub normal: Vec4,
    pub origin: Vec4,
    pub u_direction: Vec4,
    pub v_direction: Vec4,
    pub w_direction: Vec4,
}

impl Hyperplane {
    #[must_use]
    pub fn new(origin: Vec4, normal: Vec4) -> Self {
        let normal = normal.normalize();

        let basis = {
            let x_dot = Vec4::X.dot(normal).abs();
            let y_dot = Vec4::Y.dot(normal).abs();
            let z_dot = Vec4::Z.dot(normal).abs();
            let w_dot = Vec4::W.dot(normal).abs();

            if x_dot > y_dot && x_dot > z_dot && x_dot > w_dot {
                [Vec4::Y, Vec4::Z, Vec4::W]
            } else if y_dot > z_dot && y_dot > w_dot {
                [Vec4::X, Vec4::Z, Vec4::W]
            } else if z_dot > w_dot {
                [Vec4::X, Vec4::Y, Vec4::W]
            } else {
                [Vec4::X, Vec4::Y, Vec4::Z]
            }
        };

        // Use Gram-Schmidt orthogonalization to get the u, v and w vectors, which are orthogonal to the normal and each other
        let u_direction = (basis[0] - normal * normal.dot(basis[0])).normalize();
        let v_direction =
            (basis[1] - normal * normal.dot(basis[1]) - u_direction * u_direction.dot(basis[1]))
                .normalize();
        let w_direction = (basis[2]
            - normal * normal.dot(basis[2])
            - u_direction * u_direction.dot(basis[2])
            - v_direction * v_direction.dot(basis[2]))
        .normalize();

        Self {
            normal,
            origin,
            u_direction,
            v_direction,
            w_direction,
        }
    }

    #[must_use]
    pub fn project_4d(&self, p: Vec3) -> Vec4 {
        Vec4::new(
            self.origin.x
                + p.x * self.u_direction.x
                + p.y * self.v_direction.x
                + p.z * self.w_direction.x,
            self.origin.y
                + p.x * self.u_direction.y
                + p.y * self.v_direction.y
                + p.z * self.w_direction.y,
            self.origin.z
                + p.x * self.u_direction.z
                + p.y * self.v_direction.z
                + p.z * self.w_direction.z,
            self.origin.w
                + p.x * self.u_direction.w
                + p.y * self.v_direction.w
                + p.z * self.w_direction.w,
        )
    }

    #[must_use]
    pub fn project_3d(&self, p: Vec4) -> Vec3 {
        let u = (p - self.origin).dot(self.u_direction);
        let v = (p - self.origin).dot(self.v_direction);
        let w = (p - self.origin).dot(self.w_direction);

        Vec3::new(u, v, w)
    }
}

impl Vec4Operations for Hyperplane {
    fn contains(&self, pt: Vec4) -> bool {
        self.normal.dot(pt - self.origin) >= -EPSILON
    }

    fn constrain(&self, pt: Vec4) -> Vec4 {
        let t = self.normal.dot(self.origin) - self.normal.dot(pt);

        pt + self.normal * t
    }

    fn signed_distance(&self, pt: Vec4) -> f32 {
        self.normal.dot(pt - self.origin)
    }
}

pub trait HyperplaneIntersecionShape: Vec3Operations + PlaneIntersecion {}
impl<T> HyperplaneIntersecionShape for T where T: Vec3Operations + PlaneIntersecion {}

pub trait HyperplaneIntersection {
    fn intersect(&self, plane: &Hyperplane) -> Option<impl HyperplaneIntersecionShape>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hyperplane_new() {
        let origin = Vec4::new(0.0, 0.0, 0.0, 0.0);
        let normal = Vec4::new(0.3, 1.0, 0.1, 0.8).normalize();
        let plane = Hyperplane::new(origin, normal);

        assert_eq!(plane.origin, origin);
        assert_eq!(plane.normal, normal);

        // u, v and w vectors should be orthogonal to the normal
        assert!(
            plane.u_direction.dot(plane.normal).abs() < f32::EPSILON,
            "u vector is not orthogonal to normal"
        );
        assert!(
            plane.v_direction.dot(plane.normal).abs() < f32::EPSILON,
            "v vector is not orthogonal to normal. Their dot product is {}",
            plane.v_direction.dot(plane.normal).abs()
        );
        assert!(
            plane.w_direction.dot(plane.normal).abs() < f32::EPSILON,
            "w vector is not orthogonal to normal"
        );

        // u, v and w vectors should be orthogonal to each other
        assert!(
            plane.u_direction.dot(plane.v_direction).abs() < f32::EPSILON,
            "u and v vectors are not orthogonal"
        );
        assert!(
            plane.u_direction.dot(plane.w_direction).abs() < f32::EPSILON,
            "u and w vectors are not orthogonal"
        );
        assert!(
            plane.v_direction.dot(plane.w_direction).abs() < f32::EPSILON,
            "v and w vectors are not orthogonal"
        );

        // u, v and w vectors should be normalized
        assert!(
            (plane.u_direction.length_squared() - 1.0).abs() < f32::EPSILON,
            "u vector is not normalized"
        );

        assert!(
            (plane.v_direction.length_squared() - 1.0).abs() < f32::EPSILON,
            "v vector is not normalized"
        );

        assert!(
            (plane.w_direction.length_squared() - 1.0).abs() < f32::EPSILON,
            "w vector is not normalized"
        );

        // u, v and w vectors not be the same
        assert!(
            plane.u_direction != plane.v_direction,
            "u and v vectors are the same"
        );

        assert!(
            plane.u_direction != plane.w_direction,
            "u and w vectors are the same"
        );

        assert!(
            plane.v_direction != plane.w_direction,
            "v and w vectors are the same"
        );

        // u, v and w vectors should not be the same as the normal
        assert!(
            plane.u_direction != plane.normal,
            "u and normal vectors are the same"
        );

        assert!(
            plane.v_direction != plane.normal,
            "v and normal vectors are the same"
        );

        assert!(
            plane.w_direction != plane.normal,
            "w and normal vectors are the same"
        );
    }

    #[test]
    fn project_4d_and_back() {
        let origin = Vec4::new(0.0, 0.0, 0.0, 0.0);
        let normal = Vec4::new(0.3, 1.0, 0.1, 0.8).normalize();
        let plane = Hyperplane::new(origin, normal);

        let p = Vec3::new(1.0, 2.0, 3.0);
        let projected = plane.project_4d(p);
        let back = plane.project_3d(projected);

        assert!(
            (p - back).length_squared() < f32::EPSILON,
            "Projected and back-projected points are not the same"
        );
    }
}
