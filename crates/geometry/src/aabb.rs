use bevy_math::Vec3;

use crate::Vec3Operations;

#[derive(Clone, Debug)]
pub struct Aabb {
    pub center: Vec3,
    pub half_sizes: Vec3,
}

impl Aabb {
    #[must_use]
    pub fn new(center: Vec3, half_sizes: Vec3) -> Self {
        Self { center, half_sizes }
    }
}

impl Vec3Operations for Aabb {
    fn contains(&self, pt: Vec3) -> bool {
        let min = self.center - self.half_sizes;
        let max = self.center + self.half_sizes;

        pt.x >= min.x
            && pt.x <= max.x
            && pt.y >= min.y
            && pt.y <= max.y
            && pt.z >= min.z
            && pt.z <= max.z
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let min = self.center - self.half_sizes;
        let max = self.center + self.half_sizes;
        Vec3::new(
            pt.x.clamp(min.x, max.x),
            pt.y.clamp(min.y, max.y),
            pt.z.clamp(min.z, max.z),
        )
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let min = self.center - self.half_sizes;
        let max = self.center + self.half_sizes;
        let mut closest = pt;
        let mut normal = Vec3::ZERO;

        if pt.x < min.x {
            closest.x = min.x;
            normal = Vec3::new(-1.0, 0.0, 0.0);
        } else if pt.x > max.x {
            closest.x = max.x;
            normal = Vec3::new(1.0, 0.0, 0.0);
        }

        if pt.y < min.y {
            closest.y = min.y;
            normal = Vec3::new(0.0, -1.0, 0.0);
        } else if pt.y > max.y {
            closest.y = max.y;
            normal = Vec3::new(0.0, 1.0, 0.0);
        }

        if pt.z < min.z {
            closest.z = min.z;
            normal = Vec3::new(0.0, 0.0, -1.0);
        } else if pt.z > max.z {
            closest.z = max.z;
            normal = Vec3::new(0.0, 0.0, 1.0);
        }

        (closest, normal)
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        // Calculate the vector from the point to the AABB's center, and subtract the AABB's sizes.
        // Negative values indicate the point is inside the AABB.
        let delta = (pt - self.center).abs() - self.half_sizes;

        // If the point is inside the AABB, we take the maximum negative value (the farthest point inside).
        // Otherwise, we take the length of the positive components (Euclidean distance outside the AABB).
        let outside_distance = delta.max(Vec3::ZERO).length();
        let inside_distance = delta.max_element().min(0.0);

        if outside_distance > 0.0 {
            outside_distance // Outside the AABB, positive distance
        } else {
            inside_distance // Inside the AABB, negative distance
        }
    }
}
