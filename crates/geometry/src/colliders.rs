use bevy_math::Vec3;

use crate::{InfiniteCone, Plane, Sphere, Vec3Operations};

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
            Collider::Sphere(sphere) => {
                let radius = sphere.radius;
                let distance_from_point = point.length();
                let side_length = (distance_from_point.powi(2) - radius.powi(2)).sqrt();
                let angle = (radius / distance_from_point).asin();
                let distance_to_plane = (side_length * angle.cos()).abs();

                let direction = point.normalize();
                let origin = direction * (distance_to_plane - distance_from_point).abs();
                let normal = -direction;

                Plane::new(origin, normal)
            }
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
                InfiniteCone::new(vertex, direction, radius)
            }
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
