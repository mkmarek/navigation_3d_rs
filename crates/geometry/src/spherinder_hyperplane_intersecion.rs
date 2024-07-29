use std::f32::consts::PI;

use bevy_math::{Mat3, Vec2, Vec3, Vec4, Vec4Swizzles};

use crate::{
    Hyperplane, Plane, PlaneIntersecion, PlaneIntersecionShape, Spherinder,
    SpherinderHyperplanePlaneIntersection, Vec3Operations, Vec4Operations, EPSILON,
};

// This shape is a result of intersecting a spherinder with a hyperplane.
// Since the resulting shape can be challenging to describe
// by itself, it is defined by the spherinder and the hyperplane
// and all implemented operations use those two
pub struct SpherinderHyperplaneIntersecion {
    spherinder: Spherinder,
    hyperplane: Hyperplane,
}

impl SpherinderHyperplaneIntersecion {
    #[must_use]
    pub fn new(spherinder: Spherinder, hyperplane: Hyperplane) -> Self {
        Self {
            spherinder,
            hyperplane,
        }
    }
}

impl Vec3Operations for SpherinderHyperplaneIntersecion {
    fn contains(&self, _pt: Vec3) -> bool {
        todo!()
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let d4_point = self.hyperplane.project_4d(pt);
        let constrained4d = self.spherinder.constrain(d4_point);

        // If the w hyperplane normal is zero, then we can disregard the
        // w component of the constrained 4d point as the resulting intersection will be a cylinder
        // and the closest points will be along the cylinder with the same w component.
        if self.hyperplane.normal.w.abs() < EPSILON {
            self.hyperplane.project_3d(constrained4d)
        } else {
            // Calculate the plane scalar component using the normal and an origin
            let d = self.hyperplane.normal.dot(self.hyperplane.origin);

            // Calculate the w component from the hyperplane equation:
            // n.x * x + n.y * y + n.z * z + n.w * w = d
            // w = (d - n.x * x - n.y * y - n.z * z) / n.w
            let w = (d - self.hyperplane.normal.xyz().dot(constrained4d.xyz()))
                / self.hyperplane.normal.w;

            self.hyperplane.project_3d(Vec4::new(
                constrained4d.x,
                constrained4d.y,
                constrained4d.z,
                w,
            ))
        }
    }

    fn closest_point_and_normal(&self, _pt: Vec3) -> (Vec3, Vec3) {
        todo!()
    }

    fn signed_distance(&self, _pt: Vec3) -> f32 {
        todo!()
    }
}

impl PlaneIntersecion for SpherinderHyperplaneIntersecion {
    fn intersect(&self, plane: &Plane) -> Option<impl PlaneIntersecionShape> {
        // first let's calculate spherinder hyperplane intersection coefficients given a general
        // equation of an ellipsoid:
        // a1*h^2 + a4*h*s + a2*s^2 + a5*h*t + a6*s*t + a3*t^2 + a7*h + a8*s + a9*t + a10 == 0

        let a1 = self.hyperplane.w_direction.xyz().length_squared();
        let a2 = self.hyperplane.v_direction.xyz().length_squared();
        let a3 = self.hyperplane.u_direction.xyz().length_squared();

        let a4 = 2.0
            * self
                .hyperplane
                .w_direction
                .xyz()
                .dot(self.hyperplane.v_direction.xyz());
        let a5 = 2.0
            * self
                .hyperplane
                .w_direction
                .xyz()
                .dot(self.hyperplane.u_direction.xyz());
        let a6 = 2.0
            * self
                .hyperplane
                .v_direction
                .xyz()
                .dot(self.hyperplane.u_direction.xyz());

        let a7 = 2.0
            * self
                .hyperplane
                .w_direction
                .xyz()
                .dot(self.hyperplane.origin.xyz());
        let a8 = 2.0
            * self
                .hyperplane
                .v_direction
                .xyz()
                .dot(self.hyperplane.origin.xyz());
        let a9 = 2.0
            * self
                .hyperplane
                .u_direction
                .xyz()
                .dot(self.hyperplane.origin.xyz());

        let a10 = self.hyperplane.origin.xyz().length_squared()
            - self.spherinder.radius * self.spherinder.radius;

        // Spherinder x hyperplane x plane intersection coefficients given the general equation of an
        // ellipse: Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
        let px_0 = plane.origin.x;
        let py_0 = plane.origin.y;
        let pz_0 = plane.origin.z;

        let px_u = plane.u_direction.x;
        let py_u = plane.u_direction.y;
        let pz_u = plane.u_direction.z;

        let px_v = plane.v_direction.x;
        let py_v = plane.v_direction.y;
        let pz_v = plane.v_direction.z;

        let a = a3 * px_v.powi(2)
            + a6 * px_v * py_v
            + a2 * py_v.powi(2)
            + a5 * px_v * pz_v
            + a4 * py_v * pz_v
            + a1 * pz_v.powi(2);
        let b = 2.0 * a3 * px_u * px_v
            + a6 * px_v * py_u
            + a6 * px_u * py_v
            + 2.0 * a2 * py_u * py_v
            + a5 * px_v * pz_u
            + a4 * py_v * pz_u
            + a5 * px_u * pz_v
            + a4 * py_u * pz_v
            + 2.0 * a1 * pz_u * pz_v;
        let c = a3 * px_u.powi(2)
            + a6 * px_u * py_u
            + a2 * py_u.powi(2)
            + a5 * px_u * pz_u
            + a4 * py_u * pz_u
            + a1 * pz_u.powi(2);
        let d = 2.0 * a3 * px_0 * px_v
            + a6 * px_v * py_0
            + a6 * px_0 * py_v
            + 2.0 * a2 * py_0 * py_v
            + a5 * px_v * pz_0
            + a4 * py_v * pz_0
            + a5 * px_0 * pz_v
            + a4 * py_0 * pz_v
            + 2.0 * a1 * pz_0 * pz_v
            + a9 * px_v
            + a8 * py_v
            + a7 * pz_v;
        let e = 2.0 * a3 * px_0 * px_u
            + a6 * px_u * py_0
            + a6 * px_0 * py_u
            + 2.0 * a2 * py_0 * py_u
            + a5 * px_u * pz_0
            + a4 * py_u * pz_0
            + a5 * px_0 * pz_u
            + a4 * py_0 * pz_u
            + 2.0 * a1 * pz_0 * pz_u
            + a9 * px_u
            + a8 * py_u
            + a7 * pz_u;
        let f = a3 * px_0.powi(2)
            + a6 * px_0 * py_0
            + a2 * py_0.powi(2)
            + a5 * px_0 * pz_0
            + a4 * py_0 * pz_0
            + a1 * pz_0.powi(2)
            + a9 * px_0
            + a8 * py_0
            + a7 * pz_0
            + a10;

        let determinant = b.powi(2) - 4.0 * a * c;

        let center = Vec2::new(
            (2.0 * a * e - b * d) / determinant,
            (2.0 * c * d - b * e) / determinant,
        );
        let theta_angle = 0.5 * (-b).atan2(a - c) + PI / 2.0;

        let translation = Mat3::from_translation(Vec2::new(center.x, center.y));
        let rotation = Mat3::from_angle(theta_angle);

        let transform = translation * rotation;

        let k = 2.0 * (a * e.powi(2) + c * d.powi(2) - b * d * e + determinant * f);
        let h = ((a - c).powi(2) + b.powi(2)).sqrt();
        let i = k * ((a + c) + h);
        let j = k * ((a + c) - h);

        if determinant.abs() < EPSILON || i < 0.0 || j < 0.0 {
            return None;
        }

        let semi_minor_axis = -(i).sqrt() / determinant;
        let semi_major_axis = -(j).sqrt() / determinant;

        Some(SpherinderHyperplanePlaneIntersection::new(
            semi_major_axis,
            semi_minor_axis,
            center,
            transform,
        ))
    }
}
