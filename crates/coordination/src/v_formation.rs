use bevy_math::Vec3;
use geometry::Aabb;

use crate::{Formation, FormationTemplate};

const SQRT_1_OVER_2: f32 = 0.707_106_77;

pub struct VFormation {
    priority: f32,
    spacing: f32,
    agent_radius: f32,
}

impl VFormation {
    pub fn new(agent_radius: f32, spacing: f32, priority: f32) -> Self {
        Self {
            priority,
            agent_radius,
            spacing,
        }
    }
}

impl FormationTemplate for VFormation {
    fn get_priority(&self) -> f32 {
        self.priority
    }

    fn create_formation(&self, n_agents: usize) -> Formation {
        let mut positions = Vec::with_capacity(n_agents);
        let center = (n_agents as f32 / 2.0).floor();
        let agent_diameter = 2.0 * self.agent_radius;
        let axis_separation = (self.spacing + agent_diameter) * SQRT_1_OVER_2;

        for i in 0..n_agents {
            let x = (i as f32 - center) * axis_separation;
            let z = -(i as f32 - center).abs() * axis_separation;
            positions.push(Vec3::new(x, 0.0, z));
        }

        Formation::new(positions)
    }

    fn get_aabb(&self, n_agents: usize) -> Aabb {
        let agent_diameter = 2.0 * self.agent_radius;
        let axis_separation = (self.spacing + agent_diameter) * SQRT_1_OVER_2;
        let max_agents_at_each_side = (n_agents as f32 / 2.0).floor();

        let half_size_along_z = (max_agents_at_each_side * axis_separation) / 2.0;

        let (center_x, half_size_along_x) = if n_agents % 2 == 0 {
            let left_size = max_agents_at_each_side * axis_separation;
            let right_size = (max_agents_at_each_side - 1.0) * axis_separation;
            (
                -(left_size - right_size) / 2.0,
                (left_size + right_size) / 2.0,
            )
        } else {
            (0.0, max_agents_at_each_side * axis_separation)
        };

        Aabb::new(
            Vec3::new(center_x, 0.0, -half_size_along_z),
            Vec3::new(half_size_along_x, 0.0, half_size_along_z) + Vec3::splat(self.agent_radius),
        )
    }
}
