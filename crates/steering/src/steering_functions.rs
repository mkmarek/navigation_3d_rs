use bevy_math::Vec3;
use geometry::{Ray3D, SecondTangentPointResult};

use crate::TurnPlane;

const ARRIVE_MAX_FORCE_USAGE_MULTIPLIER: f32 = 0.75;
const LOOKAHEAD_TURN_MULTIPLIER: f32 = 1.1;

/// Calculates the arrive steering force for an agent.
///
/// # Arguments
///
/// * `target` - A Vec3 that represents the target position.
/// * `agent_position` - A Vec3 that represents the agent's current position.
/// * `agent_velocity` - A Vec3 that represents the agent's current velocity.
/// * `agent_mass` - A float that represents the agent's mass.
/// * `agent_max_force` - A float that represents the maximum force the agent can exert.
/// * `tolerance` - A float that represents the distance within which the agent is considered to be
/// at target
///
/// # Returns
///
/// * A Vec3 that represents the arrive steering force.
pub fn arrive(
    target: Vec3,
    agent_position: Vec3,
    agent_mass: f32,
    agent_max_force: f32,
    tolerance: f32,
) -> Vec3 {
    let displacement = target - agent_position;
    let distance = displacement.length();

    if distance > tolerance {
        let max_acceleration = agent_max_force * ARRIVE_MAX_FORCE_USAGE_MULTIPLIER / agent_mass;
        let time = (2.0 * distance / max_acceleration).sqrt();
        let max_velocity = max_acceleration * time;

        displacement.normalize() * max_velocity
    } else {
        Vec3::ZERO
    }
}

/// Calculates the seek steering force for an agent.
///
/// # Arguments
///
/// * `target` - A Vec3 that represents the target position.
/// * `agent_position` - A Vec3 that represents the agent's current position.
/// * `agent_velocity` - A Vec3 that represents the agent's current velocity.
/// * `agent_max_force` - A float that represents the maximum force the agent can exert.
///
/// # Returns
///
/// * A Vec3 that represents the seek steering force.
pub fn seek(target: Vec3, agent_position: Vec3, agent_max_force: f32, tolerance: f32) -> Vec3 {
    let displacement = target - agent_position;

    if displacement.length() < tolerance {
        return Vec3::ZERO;
    }

    displacement.normalize() * agent_max_force
}

#[derive(Debug)]
pub enum FollowPathResult {
    CurrentSegment(Vec3),
    NextSegment(Vec3, usize),
    EndOfPath(Vec3),
}

/// Follows a path defined by a sequence of points.
///
/// # Arguments
///
/// * `path` - A slice of Vec3 that represents the path to follow.
/// * `agent_position` - A Vec3 that represents the agent's current position.
/// * `agent_velocity` - A Vec3 that represents the agent's current velocity.
/// * `agent_max_turning_speed` - A float that represents the maximum turning speed of the agent.
/// * `agent_max_force` - A float that represents the maximum force the agent can exert.
/// * `agent_mass` - A float that represents the agent's mass.
///
/// # Returns
///
/// * A Vec3 that represents the steering force for the agent to follow the path.
///
/// # Description
///
/// This function calculates the steering force for an agent to follow a path defined by a sequence of points.
/// It first finds the closest point on the path to the agent. If the agent is at the end of the path, it uses the arrive behavior to stop at the last point.
/// If the agent is not able to reach the next point on the path due to its turning radius, it uses the arrive behavior to stop at the next point.
/// Otherwise, it calculates a lookahead point on the path based on the agent's turning radius and uses the seek behavior to steer towards this lookahead point.
#[allow(clippy::too_many_arguments)]
pub fn follow_path(
    path: &[Vec3],
    path_index: usize,
    agent_position: Vec3,
    agent_velocity: Vec3,
    agent_max_turning_speed: f32,
    agent_max_force: f32,
    agent_mass: f32,
    position_tolerance: f32,
) -> FollowPathResult {
    let segment = path[path_index + 1] - path[path_index];
    let segment_length = segment.length();
    let ray = Ray3D::new(path[path_index], segment);
    let parameter = ray
        .parameter_at_point(agent_position)
        .clamp(0.0, segment_length);

    if path_index == path.len() - 2 {
        if agent_position.distance(path[path_index + 1]) < position_tolerance {
            return FollowPathResult::EndOfPath(Vec3::ZERO);
        }

        return FollowPathResult::CurrentSegment(arrive(
            path[path_index + 1],
            agent_position,
            agent_mass,
            agent_max_force,
            position_tolerance,
        ));
    }

    // We'll create a turn plane and try to determine if the agent can
    // make the turn
    let turn_plane = TurnPlane::new(agent_position, path[path_index + 1], path[path_index + 2]);
    let turn_circle = turn_plane.turn_circle(agent_velocity.length(), agent_max_turning_speed);

    if parameter < segment_length / 2.0 {
        let lookahead_point = ray.at(parameter + agent_max_force);

        let seek_force = seek(
            lookahead_point,
            agent_position,
            agent_max_force,
            position_tolerance,
        );

        return FollowPathResult::CurrentSegment(seek_force);
    }

    let tangent_result = turn_plane.find_tangent_between_agent_and_turn_point(&turn_circle);

    if let SecondTangentPointResult::None = tangent_result {
        if agent_position.distance(path[path_index + 1]) < position_tolerance {
            return FollowPathResult::NextSegment(Vec3::ZERO, path_index + 1);
        }

        let arrive_force = arrive(
            path[path_index + 1],
            agent_position,
            agent_mass,
            agent_max_force,
            position_tolerance,
        );

        return FollowPathResult::CurrentSegment(arrive_force);
    }

    let min_tangent = match tangent_result {
        SecondTangentPointResult::Point(t) => t,
        SecondTangentPointResult::TwoPoints(t1, t2) => t1.min(t2),
        SecondTangentPointResult::None => unreachable!(),
    };

    // The lookeahead should be a bit larger than the turning radius of the agent
    // to compensate with any inaccuracies in computation
    let lookahead = (min_tangent * LOOKAHEAD_TURN_MULTIPLIER).max(1.0);

    let (lookahead_parameter, lookahead_index) = {
        let mut lookahead_parameter = parameter + lookahead;
        let mut lookahead_index = path_index;

        loop {
            if lookahead_index == path.len() - 2 {
                if agent_position.distance(path[lookahead_index + 1]) < position_tolerance {
                    return FollowPathResult::EndOfPath(Vec3::ZERO);
                }
                return FollowPathResult::CurrentSegment(arrive(
                    path[lookahead_index + 1],
                    agent_position,
                    agent_mass,
                    agent_max_force,
                    position_tolerance,
                ));
            }

            let segment_length = (path[lookahead_index + 1] - path[lookahead_index]).length();
            if lookahead_parameter > segment_length {
                lookahead_parameter -= segment_length;
                lookahead_index += 1;
            } else {
                break (lookahead_parameter, lookahead_index);
            }
        }
    };

    let lookahead_point = {
        let segment_direction = (path[lookahead_index + 1] - path[lookahead_index]).normalize();
        let pt = path[lookahead_index] + segment_direction * (lookahead_parameter);

        if pt.distance_squared(agent_position) <= position_tolerance * position_tolerance {
            path[lookahead_index + 1]
        } else {
            pt
        }
    };

    let seek_force = seek(
        lookahead_point,
        agent_position,
        agent_max_force,
        position_tolerance,
    );

    if lookahead_index == path_index {
        FollowPathResult::CurrentSegment(seek_force)
    } else {
        FollowPathResult::NextSegment(seek_force, lookahead_index)
    }
}

pub fn separation(
    current_position: Vec3,
    agents: &[(Vec3, f32)],
    separation_distance: f32,
) -> Vec3 {
    let mut separation_velocity = Vec3::ZERO;
    for (agent_pos, agent_radius) in agents {
        let separation_distance = agent_radius + separation_distance;
        let diff = current_position - *agent_pos;
        let distance = diff.length();
        if distance < separation_distance {
            separation_velocity += diff.normalize() * (separation_distance - distance);
        }
    }
    separation_velocity
}
