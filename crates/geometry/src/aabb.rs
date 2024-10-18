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

    pub fn merge(&mut self, other: &Self) {
        let min = self.center - self.half_sizes;
        let max = self.center + self.half_sizes;
        let other_min = other.center - other.half_sizes;
        let other_max = other.center + other.half_sizes;

        let new_min = min.min(other_min);
        let new_max = max.max(other_max);

        self.center = (new_min + new_max) / 2.0;
        self.half_sizes = (new_max - new_min) / 2.0;
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

        let distance_to_top_face = (max.y - pt.y).abs();
        let distance_to_bottom_face = (min.y - pt.y).abs();
        let distance_to_front_face = (max.z - pt.z).abs();
        let distance_to_back_face = (min.z - pt.z).abs();

        if distance_to_top_face.min(distance_to_bottom_face)
            < distance_to_front_face.min(distance_to_back_face)
        {
            if distance_to_top_face < distance_to_bottom_face {
                (Vec3::new(pt.x, max.y, pt.z), Vec3::Y)
            } else {
                (Vec3::new(pt.x, min.y, pt.z), -Vec3::Y)
            }
        } else if distance_to_front_face < distance_to_back_face {
            (Vec3::new(pt.x, pt.y, max.z), Vec3::Z)
        } else {
            (Vec3::new(pt.x, pt.y, min.z), -Vec3::Z)
        }
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
