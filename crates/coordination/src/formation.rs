use bevy_math::Vec3;
use geometry::Aabb;

pub struct Formation {
    positions: Vec<Vec3>,
}

impl Formation {
    pub fn new(positions: Vec<Vec3>) -> Self {
        Self { positions }
    }

    pub fn get_positions(&self) -> &[Vec3] {
        &self.positions
    }

    pub fn get_bounds(&self) -> Aabb {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);

        for &position in self.positions.iter() {
            min = min.min(position);
            max = max.max(position);
        }

        let center = (min + max) / 2.0;
        let half_sizes = (max - min) / 2.0;

        Aabb::new(center, half_sizes)
    }

    pub fn scale(&mut self, scale: f32) {
        for position in self.positions.iter_mut() {
            *position *= scale;
        }
    }

    pub fn get_closest_pairs(&self, other: &Self) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        for (i, position) in self.positions.iter().enumerate() {
            let mut closest = f32::INFINITY;
            let mut closest_index = 0;
            for (j, other_position) in other.positions.iter().enumerate() {
                let distance = position.distance_squared(*other_position);
                if distance < closest {
                    closest = distance;
                    closest_index = j;
                }
            }
            pairs.push((i, closest_index));
        }
        pairs
    }
}
