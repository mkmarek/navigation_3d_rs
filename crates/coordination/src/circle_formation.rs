use std::f32::consts::TAU;

use crate::{Formation, FormationTemplate};

pub struct CircleFormation {
    radius: f32,
    priority: f32,
}

impl CircleFormation {
    pub fn new(radius: f32, priority: f32) -> Self {
        Self { radius, priority }
    }
}

impl FormationTemplate for CircleFormation {
    fn get_priority(&self) -> f32 {
        self.priority
    }

    fn create_formation(&self, n_agents: usize) -> Formation {
        let mut positions = Vec::with_capacity(n_agents);
        for i in 0..n_agents {
            let angle = i as f32 / n_agents as f32 * TAU;
            let x = angle.cos();
            let z = angle.sin();
            positions.push(bevy_math::Vec3::new(x, 0.0, z) * 100.0);
        }

        Formation::new(positions)
    }
}
