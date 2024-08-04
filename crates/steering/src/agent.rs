use bevy_math::{Quat, Vec3};

#[allow(clippy::too_many_arguments)]
pub fn update_agent_on_path(
    velocity: Vec3,
    rotation: Quat,
    max_turn_speed: f32,
    max_speed: f32,
    max_force: f32,
    mass: f32,
    force: Vec3,
    delta_time: f32,
) -> (Vec3, Quat) {
    if force.length_squared() < f32::EPSILON {
        return (velocity, rotation);
    }

    let velocity_sq = velocity.length_squared();
    let current_heading = rotation.mul_vec3(Vec3::X).normalize();

    let force_angle = force.angle_between(current_heading);
    let max_acceleration = max_force / mass;

    // The velocity tolerance is how much the velocity can change in 3 frames
    let velocity_tolerance = max_acceleration * delta_time * 3.0;

    // The angle tolerance is how much the angle can change in 3 frames
    let angle_tolerance = max_turn_speed * delta_time * 3.0;

    // If the velocity is close to zero and the heading is different enough from the current
    // rotation, we'll rotate the agent to face the direction of the desired force
    if velocity_sq <= velocity_tolerance * velocity_tolerance && force_angle >= angle_tolerance {
        let mut rotation_increment = Quat::from_rotation_arc(current_heading, force.normalize());
        rotation_increment = Quat::IDENTITY.slerp(
            rotation_increment,
            (max_turn_speed / force_angle * delta_time).clamp(0.0, 1.0),
        );

        return (Vec3::ZERO, (rotation_increment * rotation));
    }

    let new_rotation = if force_angle >= angle_tolerance {
        let mut rotation_increment = Quat::from_rotation_arc(current_heading, force.normalize());
        rotation_increment = Quat::IDENTITY.slerp(
            rotation_increment,
            (max_turn_speed / force_angle * delta_time).clamp(0.0, 1.0),
        );

        rotation_increment * rotation
    } else {
        let rotation_increment = Quat::from_rotation_arc(current_heading, force.normalize());
        rotation_increment * rotation
    };

    let new_heading = new_rotation.mul_vec3(Vec3::X).normalize();
    let acceleration = force.clamp_length_max(max_force) / mass;
    let velocity_change = new_heading.dot(acceleration) * delta_time;

    let new_velocity_magnitude = (velocity.length() + velocity_change).min(max_speed);
    let new_velocity = new_heading * new_velocity_magnitude;

    (new_velocity, new_rotation)
}
