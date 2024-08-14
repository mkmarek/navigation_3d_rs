use bevy_math::Vec3;

use crate::{Circle, Circle3d, Plane, PlaneIntersecion, PlaneIntersecionShape, Vec3Operations};

// Defines a 3D sphere with a radius and origin.
#[derive(Clone, Debug)]
pub struct Sphere {
    pub radius: f32,
    pub origin: Vec3,
}

pub enum SphereSphereIntersection {
    Inside,
    Outside,
    Intersecting(Circle3d),
}

impl Sphere {
    #[must_use]
    pub fn new(radius: f32, origin: Vec3) -> Self {
        Self { radius, origin }
    }

    #[must_use]
    pub fn get_secant_plane(&self, point: Vec3) -> Plane {
        let relative_pt = point - self.origin;

        let radius = self.radius;
        let distance_from_point = relative_pt.length();
        let side_length = (distance_from_point.powi(2) - radius.powi(2)).sqrt();
        let angle = (radius / distance_from_point).asin();
        let distance_to_plane = (side_length * angle.cos()).abs();

        let direction = relative_pt.normalize();
        let origin = direction * (distance_to_plane - distance_from_point).abs();
        let normal = -direction;

        Plane::new(origin + self.origin, normal)
    }

    pub fn intersect_sphere(&self, other: &Sphere) -> SphereSphereIntersection {
        // Calculate the distance between the centers of the two spheres
        let d = self.origin.distance(other.origin);

        // Check for intersection conditions
        if d > self.radius + other.radius {
            return SphereSphereIntersection::Outside;
        }

        if d < (self.radius - other.radius).abs() {
            return SphereSphereIntersection::Inside;
        }

        // Calculate the radius of the intersection circle
        let r1_sq = self.radius.powi(2);
        let r2_sq = other.radius.powi(2);
        let d_sq = d.powi(2);

        let a = (r1_sq - r2_sq + d_sq) / (2.0 * d);
        let h_sq = r1_sq - a.powi(2);

        if h_sq < 0.0 {
            return SphereSphereIntersection::Inside;
        }

        let h = h_sq.sqrt();
        let circle_radius = h;

        // Calculate the center of the intersection circle
        let direction = (other.origin - self.origin) / d;
        let center = self.origin + direction * a;

        SphereSphereIntersection::Intersecting(Circle3d::new(circle_radius, center, direction))
    }

    pub fn intersect_plane(&self, plane: &Plane) -> Option<Circle> {
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
    fn intersect(&self, plane: &Plane) -> Option<impl PlaneIntersecionShape> {
        self.intersect_plane(plane)
    }
}

pub trait SphereIntersectionShape: Vec3Operations {}
impl<T> SphereIntersectionShape for T where T: Vec3Operations {}

pub trait SphereIntersection {
    fn intersect(&self, plane: &Sphere) -> Option<impl SphereIntersectionShape>;
}

impl SphereIntersection for Sphere {
    // https://mathworld.wolfram.com/Sphere-SphereIntersection.html
    fn intersect(&self, other: &Sphere) -> Option<impl SphereIntersectionShape> {
        match self.intersect_sphere(other) {
            SphereSphereIntersection::Inside => None,
            SphereSphereIntersection::Outside => None,
            SphereSphereIntersection::Intersecting(circle) => Some(circle),
        }
    }
}
