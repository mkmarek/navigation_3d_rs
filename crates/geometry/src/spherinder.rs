use bevy_math::{Vec4, Vec4Swizzles};

use crate::{
    Hyperplane, HyperplaneIntersecionShape, HyperplaneIntersection,
    SpherinderHyperplaneIntersecion, Vec4Operations,
};

// A four dimensional shape that is a sphere in three dimensions extended into the fourth dimension.
// This shape assumes that it extends into the w dimension infinitely.
#[derive(Clone, Debug)]
pub struct Spherinder {
    pub origin: Vec4,
    pub radius: f32,
}

impl Spherinder {
    #[must_use]
    pub fn new(origin: Vec4, radius: f32) -> Self {
        Self { origin, radius }
    }
}

impl Vec4Operations for Spherinder {
    fn contains(&self, pt: Vec4) -> bool {
        let relative_pt = pt - self.origin;
        let xyz = relative_pt.xyz();

        xyz.length_squared() <= self.radius * self.radius
    }

    fn constrain(&self, pt: Vec4) -> Vec4 {
        let relative_pt = pt - self.origin;
        let xyz = relative_pt.xyz();
        let w = relative_pt.w;

        if xyz.length_squared() < self.radius * self.radius {
            return pt;
        }

        let new_xyz = xyz.normalize() * self.radius;

        self.origin + Vec4::new(new_xyz.x, new_xyz.y, new_xyz.z, w)
    }

    fn signed_distance(&self, pt: Vec4) -> f32 {
        let relative_pt = pt - self.origin;
        let xyz = relative_pt.xyz();

        xyz.length() - self.radius
    }
}

impl HyperplaneIntersection for Spherinder {
    fn intersect(&self, plane: &Hyperplane) -> Option<impl HyperplaneIntersecionShape> {
        Some(SpherinderHyperplaneIntersecion::new(
            self.clone(),
            plane.clone(),
        ))
    }
}
