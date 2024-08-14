use bevy_math::Vec3;

use crate::Vec3Operations;

#[derive(Debug)]
pub struct Circle3d {
    pub radius: f32,
    pub origin: Vec3,
    pub normal: Vec3,
}

impl Circle3d {
    #[must_use]
    pub fn new(radius: f32, origin: Vec3, normal: Vec3) -> Self {
        Self {
            radius,
            origin,
            normal,
        }
    }
}

impl Vec3Operations for Circle3d {
    fn contains(&self, pt: Vec3) -> bool {
        let relative_pt = pt - self.origin;
        if (self.normal.dot(relative_pt)).abs() < 0.0001 {
            relative_pt.length_squared() <= self.radius * self.radius
        } else {
            false
        }
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let relative_pt = pt - self.origin;
        let relative_pt = if (self.normal.dot(relative_pt)).abs() < 0.0001 {
            relative_pt
        } else {
            let projected_offset = self.normal * self.normal.dot(relative_pt);
            relative_pt - projected_offset
        };

        if relative_pt.length_squared() > self.radius * self.radius {
            self.origin + relative_pt.normalize() * self.radius
        } else {
            pt
        }
    }

    fn closest_point_and_normal(&self, _pt: Vec3) -> (Vec3, Vec3) {
        todo!()
    }

    fn signed_distance(&self, _pt: Vec3) -> f32 {
        todo!()
    }
}
