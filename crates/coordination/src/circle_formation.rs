use crate::Formation;

pub struct CircleFormation {}

impl Formation for CircleFormation {
    fn get_positions(&self, n_agents: usize) -> Vec<bevy_math::Vec3> {
        let mut positions = Vec::with_capacity(n_agents);
        for i in 0..n_agents {
            let angle = i as f32 / n_agents as f32 * std::f32::consts::TAU;
            let x = angle.cos();
            let z = angle.sin();
            positions.push(bevy_math::Vec3::new(x, 0.0, z) * 100.0);
        }
        positions
    }
}
