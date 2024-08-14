use bevy_math::Vec3;
use geometry::{colliders::Collider, Vec3Operations};

use crate::{Agent3D, Plane};

pub struct VelocityObstacle3D {
    pub relative_position: Vec3,
    pub relative_velocity: Vec3,
    pub shape: Collider,
    pub cutoff_shape: Collider,
    pub agent_velocity: Vec3,
    pub time_horizon: f32,
    pub responsibility: f32,
}

impl VelocityObstacle3D {
    #[must_use]
    pub fn new(agent_self: &Agent3D, agent_other: &Agent3D, time_horizon: f32) -> Self {
        let shape = agent_self.shape.minkowski_sum(&agent_other.shape);
        let cutoff_shape = shape.scale(1.0 / time_horizon);

        let relative_position = agent_other.position - agent_self.position;
        let relative_velocity = agent_self.velocity - agent_other.velocity;
        let agent_velocity = agent_self.velocity;
        let total_responsibility = agent_self.responsibility + agent_other.responsibility;
        let agent_self_responsibility = agent_self.responsibility / total_responsibility;

        Self {
            relative_position,
            relative_velocity,
            shape,
            cutoff_shape,
            agent_velocity,
            time_horizon,
            responsibility: agent_self_responsibility,
        }
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn orca_plane(&self, time_step: f32) -> Plane {
        // Vector from cutoff center to relative velocity.
        let from_cutoff_center_to_relative_velocity =
            self.relative_velocity - self.relative_position / self.time_horizon;

        let (u, normal) = if self.shape.contains(self.relative_position) {
            let from_cutoff_center_to_relative_velocity =
                self.relative_velocity - self.relative_position / time_step;

            let time_step_cutoff_shape = self.shape.scale(1.0 / time_step);

            let (p, normal) = time_step_cutoff_shape
                .closest_point_and_normal(from_cutoff_center_to_relative_velocity);

            // p is relative to cutoff center, we need to make it relative to relative_velocity
            let u = p + (self.relative_position / time_step) - self.relative_velocity;

            (u, normal)
        } else {
            // We'll create a plane centered at the cutoff sphere with a normal pointing towards zero.
            let is_in_front_of_secant_plane = {
                let from_cutoff_center_to_relative_position =
                    self.relative_position - self.relative_position / self.time_horizon;

                let secant_plane = self
                    .cutoff_shape
                    .get_secant_plane(from_cutoff_center_to_relative_position);

                let (p, _) = self
                    .cutoff_shape
                    .closest_point_and_normal(from_cutoff_center_to_relative_velocity);

                secant_plane.contains(p)
            };

            if is_in_front_of_secant_plane {
                // If the relative velocity is in front of that plane, then we project on cutoff
                // shape
                let (p, normal) = self
                    .cutoff_shape
                    .closest_point_and_normal(from_cutoff_center_to_relative_velocity);

                // p is relative to cutoff center, we need to make it relative to relative_velocity
                let u = p + (self.relative_position / self.time_horizon) - self.relative_velocity;

                (u, normal)
            } else {
                let (pt, normal) = self
                    .shape
                    .extend_cone(-self.relative_position)
                    .closest_point_and_normal(self.relative_velocity - self.relative_position);

                let u = pt + self.relative_position - self.relative_velocity;

                (u, normal)
            }
        };

        Plane::new(self.agent_velocity + self.responsibility * u, normal)
    }
}
