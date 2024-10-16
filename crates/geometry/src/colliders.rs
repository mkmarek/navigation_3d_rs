use bevy_math::Vec3;

use crate::{Aabb, Cone, Plane, Sphere, Vec3Operations};

#[derive(Clone, Debug)]
pub enum Collider {
    Sphere(Sphere),
    Aabb(Aabb),
}

impl Collider {
    #[must_use]
    pub fn new_sphere(radius: f32) -> Self {
        Collider::Sphere(Sphere::new(radius, Vec3::ZERO))
    }

    #[must_use]
    pub fn new_aabb(center: Vec3, half_sizes: Vec3) -> Self {
        Collider::Aabb(Aabb::new(center, half_sizes))
    }

    #[must_use]
    pub fn get_secant_plane(&self, point: Vec3) -> Plane {
        match self {
            Collider::Sphere(sphere) => sphere.get_secant_plane(point),
            Collider::Aabb(_aabb) => todo!(),
        }
    }

    pub fn minkowski_sum(&self, other: &Collider) -> Collider {
        match (self, other) {
            (Collider::Sphere(sphere1), Collider::Sphere(sphere2)) => {
                let radius = sphere1.radius + sphere2.radius;
                let origin = sphere1.origin - sphere2.origin;
                Collider::Sphere(Sphere::new(radius, origin))
            }
            (Collider::Sphere(sphere), Collider::Aabb(aabb)) => Collider::Aabb(Aabb::new(
                sphere.origin - aabb.center,
                aabb.half_sizes + Vec3::splat(sphere.radius),
            )),
            (Collider::Aabb(aabb), Collider::Sphere(sphere)) => Collider::Aabb(Aabb::new(
                aabb.center - sphere.origin,
                aabb.half_sizes + Vec3::splat(sphere.radius),
            )),
            (Collider::Aabb(aabb1), Collider::Aabb(aabb2)) => Collider::Aabb(Aabb::new(
                aabb1.center - aabb2.center,
                aabb1.half_sizes + aabb2.half_sizes,
            )),
        }
    }

    pub fn scale(&self, scale: f32) -> Collider {
        match self {
            Collider::Sphere(sphere) => {
                let radius = sphere.radius * scale;
                Collider::Sphere(Sphere::new(radius, sphere.origin))
            }
            Collider::Aabb(aabb) => {
                let half_sizes = aabb.half_sizes * scale;
                Collider::Aabb(Aabb::new(aabb.center, half_sizes))
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
            Collider::Aabb(_) => todo!(),
        }
    }

    pub fn bounding_sphere(&self) -> Sphere {
        match self {
            Collider::Sphere(sphere) => sphere.clone(),
            Collider::Aabb(aabb) => {
                let radius = aabb.half_sizes.length();
                Sphere::new(radius, aabb.center)
            }
        }
    }

    pub fn is_symmetric(&self) -> bool {
        match self {
            Collider::Sphere(_) => true,
            Collider::Aabb(aabb) => {
                aabb.half_sizes.x == aabb.half_sizes.y && aabb.half_sizes.y == aabb.half_sizes.z
            }
        }
    }
}

impl Vec3Operations for Collider {
    fn contains(&self, pt: Vec3) -> bool {
        match self {
            Collider::Sphere(sphere) => sphere.contains(pt),
            Collider::Aabb(aabb) => aabb.contains(pt),
        }
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        match self {
            Collider::Sphere(sphere) => sphere.constrain(pt),
            Collider::Aabb(aabb) => aabb.constrain(pt),
        }
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        match self {
            Collider::Sphere(sphere) => sphere.closest_point_and_normal(pt),
            Collider::Aabb(aabb) => aabb.closest_point_and_normal(pt),
        }
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        match self {
            Collider::Sphere(sphere) => sphere.signed_distance(pt),
            Collider::Aabb(aabb) => aabb.signed_distance(pt),
        }
    }
}
