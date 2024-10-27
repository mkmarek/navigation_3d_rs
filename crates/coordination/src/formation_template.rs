use bevy_math::Vec3;
use geometry::{colliders::Collider, Aabb};
use orca::{optimize_velocity_3d, Agent3D, FormationVelocityObstacle3D};

use crate::{expectation_maximization::expectation_maximization, Formation};

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

    // Gets the AABB bounding box for the formation
    // n_agents: The number of agents in the formations
    // Returns: The AABB bounding box of the formation
    fn get_aabb(&self, n_agents: usize) -> Aabb;
}

pub struct FormationTemplateSet<'a>(Vec<&'a dyn FormationTemplate>);

impl<'a> FromIterator<&'a dyn FormationTemplate> for FormationTemplateSet<'a> {
    fn from_iter<T: IntoIterator<Item = &'a dyn FormationTemplate>>(iter: T) -> Self {
        Self(iter.into_iter().collect())
    }
}

impl<'a> FormationTemplateSet<'a> {
    pub fn from_slice<'b>(templates: &'b [&'a dyn FormationTemplate]) -> Self {
        Self(templates.to_vec())
    }

    // Each formation is evaluated by a fitness function E(F) = p_f * (v_f.dot(v_pref)))
    // Where: v_pref is the preferred velocity of the formation
    //        v_f is the collision-free velocity of the formation
    //        p_f is the priority of the formation.
    //
    // The priority of the formation is given by the formula:
    // p_f = a_1 * p_1 + a_2 * p_2 + ... + a_n * p_n - gamma * sigma
    // Where: p_i is the priority of the i-th formation
    //        a_i is the weight of the i-th formation (For template formations it's always 1
    //        for their respective formation)
    //        gamma is the deformation penalty multiplier
    //        sigma is the standard deviation from the template formation (Zero for template
    //        formations)
    //        n is the number of formation templates
    //
    // The formation with the highest fitness function is selected
    #[allow(clippy::too_many_arguments)]
    pub fn get_best_formation_and_velocity(
        &self,
        current_formation: &[Vec3],
        preffered_velocity: Vec3,
        maximum_velocity: f32,
        deformation_penalty_multiplier: f32,
        obtacles: &[Agent3D],
        obstacle_avoidance_time_horizon: f32,
        number_of_yaw_samples: u16,
        number_of_pitch_samples: u16,
        max_steps_for_em: usize,
    ) -> (Formation, Vec3) {
        let mut best_formation = None;
        let mut best_velocity = None;
        let mut best_fitness = f32::NEG_INFINITY;

        // First evaluate the fitness of each template formation
        for template in &self.0 {
            let template_aabb = template.get_aabb(current_formation.len());

            let formation_agent = Agent3D::new(
                template_aabb.center,
                preffered_velocity,
                Collider::new_aabb(Vec3::ZERO, template_aabb.half_sizes),
            );

            let orca_planes = obtacles
                .iter()
                .filter_map(|obstacle| {
                    let vo = FormationVelocityObstacle3D::new(
                        &formation_agent,
                        obstacle,
                        obstacle_avoidance_time_horizon,
                    );

                    vo.orca_plane(number_of_yaw_samples, number_of_pitch_samples, 0.0)
                })
                .collect::<Vec<_>>();

            let optimal_velocity = if orca_planes.is_empty() {
                preffered_velocity
            } else {
                optimize_velocity_3d(preffered_velocity, maximum_velocity, &orca_planes)
            };

            let fitness = template.get_priority() * optimal_velocity.dot(preffered_velocity);

            if fitness > best_fitness {
                best_fitness = fitness;
                best_formation = Some(template.create_formation(current_formation.len()));
                best_velocity = Some(optimal_velocity);
            }
        }

        // Now evaluate the fitness of the current formation
        {
            let (formation_aabb, center) = {
                let mut min = Vec3::splat(f32::INFINITY);
                let mut max = Vec3::splat(f32::NEG_INFINITY);

                for position in current_formation {
                    min = min.min(*position);
                    max = max.max(*position);
                }

                (
                    Collider::new_aabb(Vec3::ZERO, (max - min) / 2.0),
                    (min + max) / 2.0,
                )
            };

            let formation_agent = Agent3D::new(center, preffered_velocity, formation_aabb);

            let orca_planes = obtacles
                .iter()
                .filter_map(|obstacle| {
                    FormationVelocityObstacle3D::new(
                        &formation_agent,
                        obstacle,
                        obstacle_avoidance_time_horizon,
                    )
                    .orca_plane(
                        number_of_yaw_samples,
                        number_of_pitch_samples,
                        0.0,
                    )
                })
                .collect::<Vec<_>>();

            let optimal_velocity =
                optimize_velocity_3d(preffered_velocity, maximum_velocity, &orca_planes);

            let formation_templates = self
                .0
                .iter()
                .map(|template| template.create_formation(current_formation.len()))
                .collect::<Vec<_>>();

            let formation_templates_ref = formation_templates
                .iter()
                .map(|e| e.get_positions())
                .collect::<Vec<_>>();

            let (coefficients, std_dev) = expectation_maximization(
                current_formation,
                &formation_templates_ref,
                max_steps_for_em,
            );

            let priority = coefficients
                .iter()
                .zip(self.0.iter())
                .map(|(c, t)| c * t.get_priority())
                .sum::<f32>()
                - deformation_penalty_multiplier * std_dev;

            let fitness = priority * optimal_velocity.dot(preffered_velocity);

            if fitness > best_fitness + 1e-3 {
                best_formation = Some(Formation::new(current_formation.to_vec()));
                best_velocity = Some(optimal_velocity);
            }
        }

        let best_form = best_formation.expect("No formation found");
        let best_vel = best_velocity.expect("No velocity found");

        (best_form, best_vel)
    }
}
