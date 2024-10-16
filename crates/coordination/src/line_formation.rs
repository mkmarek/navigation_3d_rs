use bevy_math::Vec3;
use geometry::Aabb;

use crate::{Formation, FormationTemplate};

pub struct LineFormation {
    priority: f32,
    spacing: f32,
    agent_radius: f32,
}

impl LineFormation {
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
}

impl FormationTemplate for LineFormation {
    fn get_priority(&self) -> f32 {
        self.priority
    }

    fn create_formation(&self, n_agents: usize) -> Formation {
        assert!(n_agents > 0);

        let mut positions = Vec::with_capacity(n_agents);
        let center = (n_agents as f32 / 2.0).floor();
        let agent_diameter = 2.0 * self.agent_radius;

        for i in 0..n_agents {
            let x = (i as f32 - center) * (self.spacing + agent_diameter);
            positions.push(Vec3::new(x, 0.0, 0.0));
        }

        Formation::new(positions)
    }

    fn get_aabb(&self, n_agents: usize) -> Aabb {
        assert!(n_agents > 0);
        let agent_diameter = 2.0 * self.agent_radius;
        let half_size_from_center =
            (n_agents as f32 / 2.0) * (self.spacing + agent_diameter) - self.spacing / 2.0;

        let half_sizes = Vec3::new(half_size_from_center, self.agent_radius, self.agent_radius);

        let center = if n_agents % 2 == 0 {
            -(self.agent_radius + self.spacing / 2.0)
        } else {
            0.0
        };

        Aabb::new(Vec3::new(center, 0.0, 0.0), half_sizes)
    }
}
