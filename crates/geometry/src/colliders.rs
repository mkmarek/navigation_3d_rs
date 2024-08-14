use bevy_math::Vec3;

use crate::{Cone, Plane, Sphere, Vec3Operations};

#[derive(Clone, Debug)]
pub enum Collider {
    Sphere(Sphere),
}

impl Collider {
    #[must_use]
    pub fn new_sphere(radius: f32) -> Self {
        Collider::Sphere(Sphere::new(radius, Vec3::ZERO))
    }

    #[must_use]
    pub fn get_secant_plane(&self, point: Vec3) -> Plane {
        match self {
            Collider::Sphere(sphere) => sphere.get_secant_plane(point),
        }
    }

    pub fn minkowski_sum(&self, other: &Collider) -> Collider {
        match (self, other) {
            (Collider::Sphere(sphere1), Collider::Sphere(sphere2)) => {
                let radius = sphere1.radius + sphere2.radius;
                let origin = sphere1.origin - sphere2.origin;
                Collider::Sphere(Sphere::new(radius, origin))
            }
        }
    }

    pub fn scale(&self, scale: f32) -> Collider {
        match self {
            Collider::Sphere(sphere) => {
                let radius = sphere.radius * scale;
                Collider::Sphere(Sphere::new(radius, sphere.origin))
            }
        }
    }

    #[must_use]
    pub fn extend_cone(&self, vertex: Vec3) -> impl Vec3Operations {
        match self {
            Collider::Sphere(sphere) => {
                let radius = sphere.radius;
                let direction = -vertex;
                Cone::infinite(vertex, direction, radius)
            }
        }
    }

    pub fn bounding_sphere(&self) -> Sphere {
        match self {
            Collider::Sphere(sphere) => sphere.clone(),
        }
    }
}

impl Vec3Operations for Collider {
    fn contains(&self, pt: bevy_math::Vec3) -> bool {
        match self {
            Collider::Sphere(sphere) => sphere.contains(pt),
        }
    }

    fn constrain(&self, pt: bevy_math::Vec3) -> bevy_math::Vec3 {
        match self {
            Collider::Sphere(sphere) => sphere.constrain(pt),
        }
    }

    fn closest_point_and_normal(&self, pt: bevy_math::Vec3) -> (bevy_math::Vec3, bevy_math::Vec3) {
        match self {
            Collider::Sphere(sphere) => sphere.closest_point_and_normal(pt),
        }
    }

    fn signed_distance(&self, pt: bevy_math::Vec3) -> f32 {
        match self {
            Collider::Sphere(sphere) => sphere.signed_distance(pt),
        }
    }
}
