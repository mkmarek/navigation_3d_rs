use bevy_math::Vec3;
use geometry::Aabb;

#[derive(Clone, Debug)]
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

    pub fn get_bounds(&self, agent_radius: f32) -> Aabb {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);

        for &position in self.positions.iter() {
            min = min.min(position);
            max = max.max(position);
        }

        min -= Vec3::splat(agent_radius);
        max += Vec3::splat(agent_radius);

        let center = (min + max) / 2.0;
        let half_sizes = (max - min) / 2.0;

        Aabb::new(center, half_sizes)
    }

    pub fn scale(&mut self, scale: f32) {
        for position in self.positions.iter_mut() {
            *position *= scale;
        }
    }
}
