use bevy_math::Vec3;

pub trait Formation {
    fn get_positions(&self, n_agents: usize) -> Vec<Vec3>;
}
