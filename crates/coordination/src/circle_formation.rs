use std::f32::consts::TAU;

use bevy_math::Vec3;
use geometry::Aabb;

use crate::{Formation, FormationTemplate};

pub struct CircleFormation {
    agent_radius: f32,
    spacing: f32,
    priority: f32,
}

impl CircleFormation {
    pub fn new(agent_radius: f32, spacing: f32, priority: f32) -> Self {
        assert!(agent_radius > 0.0);
        assert!(spacing >= 0.0);
        assert!(priority > 0.0);

        Self {
            agent_radius,
            spacing,
            priority,
        }
    }

    fn get_radius(&self, n_agents: usize) -> f32 {
        assert!(n_agents > 0);

        if n_agents <= 1 {
            return 0.0;
        }

        let agent_diameter = 2.0 * self.agent_radius;
        ((agent_diameter + self.spacing) * n_agents as f32 / TAU).max(agent_diameter)
    }
}

impl FormationTemplate for CircleFormation {
    fn get_priority(&self) -> f32 {
        self.priority
    }

    fn create_formation(&self, n_agents: usize) -> Formation {
        assert!(n_agents > 0);

        let mut positions = Vec::with_capacity(n_agents);
        let radius = self.get_radius(n_agents);

        for i in 0..n_agents {
            let angle = i as f32 / n_agents as f32 * TAU;
            let x = angle.cos();
            let z = angle.sin();
            positions.push(bevy_math::Vec3::new(x, 0.0, z) * radius);
        }

        Formation::new(positions)
    }

    fn get_aabb(&self, n_agents: usize) -> Aabb {
        assert!(n_agents > 0);

        let radius = self.get_radius(n_agents) + self.agent_radius;

        let half_sizes = Vec3::new(radius, self.agent_radius, radius);

        Aabb::new(Vec3::ZERO, half_sizes)
    }
}
