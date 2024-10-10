use crate::Formation;

pub trait FormationTemplate {
    // Get the positions of the agents in the formation
    // The zeroth position is always the center of the formation
    //
    // n_agents: The number of agents in the formation
    // Returns: A vector of Vec3 positions
    fn create_formation(&self, n_agents: usize) -> Formation;

    // Get the priority of the formation
    // Returns: A float representing the priority of the formation
    fn get_priority(&self) -> f32;
}
