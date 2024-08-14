use bevy_math::Vec3;

pub trait Formation {
    fn get_positions(&self, n_agents: usize) -> Vec<Vec3>;
}

pub trait AgentInFormation<ID>
where
    ID: Copy,
{
    fn get_id(&self) -> ID;
    fn get_position(&self) -> Vec3;
}

pub fn select_formation_positions<AgentID>(
    agents: &[impl AgentInFormation<AgentID>],
    formation: &impl Formation,
) -> Vec<(AgentID, Vec3)>
where
    AgentID: Copy,
{
    let n_agents = agents.len();
    let formation_positions = formation.get_positions(n_agents);

    let mut result = Vec::with_capacity(n_agents);
    for (i, agent) in agents.iter().enumerate() {
        let formation_position = formation_positions[i];
        result.push((agent.get_id(), formation_position));
    }

    result
}
