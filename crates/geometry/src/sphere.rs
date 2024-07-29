use bevy_math::Vec3;

use crate::{Circle, PlaneIntersecion, PlaneIntersecionShape, Vec3Operations};

// Defines a 3D sphere with a radius and origin.
#[derive(Clone, Debug)]
pub struct Sphere {
    pub radius: f32,
    pub origin: Vec3,
}

impl Sphere {
    #[must_use]
    pub fn new(radius: f32, origin: Vec3) -> Self {
        Self { radius, origin }
    }
}

pub trait SphereMinkowskiSum {
    fn minkowski_sum(&self, other: &Self) -> impl Vec3Operations;
}

impl SphereMinkowskiSum for Sphere {
    fn minkowski_sum(&self, other: &Self) -> impl Vec3Operations {
        let radius = self.radius + other.radius;
        let origin = self.origin + other.origin;

        Sphere::new(radius, origin)
    }
}

impl Vec3Operations for Sphere {
    fn contains(&self, pt: Vec3) -> bool {
        let relative_pt = pt - self.origin;

        relative_pt.length_squared() <= self.radius * self.radius
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let relative_pt = pt - self.origin;

        if relative_pt.length_squared() <= self.radius * self.radius {
            return pt;
        }

        self.origin + relative_pt.normalize() * self.radius
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let relative_pt = pt - self.origin;

        if relative_pt.length_squared() <= self.radius * self.radius {
            return (pt, relative_pt.normalize());
        }

        (
            self.origin + relative_pt.normalize() * self.radius,
            relative_pt.normalize(),
        )
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let relative_pt = pt - self.origin;

        relative_pt.length() - self.radius
    }
}

impl PlaneIntersecion for Sphere {
    fn intersect(&self, plane: &crate::Plane) -> Option<impl PlaneIntersecionShape> {
        let (plane_pt, _) = plane.closest_point_and_normal(self.origin);
        let origin_distance = (plane_pt - self.origin).length();

        if origin_distance > self.radius {
            return None;
        }

        let radius = (self.radius * self.radius - origin_distance * origin_distance).sqrt();

        let plane_pt_2d = plane.project_2d(plane_pt);

        Some(Circle::new(radius, plane_pt_2d))
    }
}
