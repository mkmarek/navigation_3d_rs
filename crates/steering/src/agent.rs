use bevy_math::{Quat, Vec3};

#[allow(clippy::too_many_arguments)]
pub fn update_agent_on_path(
    velocity: Vec3,
    rotation: Quat,
    max_turn_speed: f32,
    max_speed: f32,
    max_force: f32,
    mass: f32,
    desired_velocity: Vec3,
    delta_time: f32,
) -> (Vec3, Quat) {
    if (velocity - desired_velocity).length_squared() < f32::EPSILON {
        return (velocity, rotation);
    }

    let velocity_sq = velocity.length_squared();
    let current_heading = rotation.mul_vec3(Vec3::X).normalize();
    let max_acceleration = max_force / mass;

    // The velocity tolerance is how much the velocity can change in 2 frames
    let velocity_tolerance = max_acceleration * delta_time * 2.0;

    // The angle tolerance is how much the angle can change in 2 frames
    let angle_tolerance = (max_turn_speed * delta_time * 2.0).min(0.001);

    let desired_heading = if desired_velocity.length_squared() > f32::EPSILON {
        Some(desired_velocity.normalize())
    } else {
        None
    };

    // If the velocity is close to zero and the heading is different enough from the current
    // rotation, we'll rotate the agent to face the direction of the desired force
    if let Some(desired_heading) = desired_heading {
        let force_angle = current_heading.angle_between(desired_heading);
        if velocity_sq <= velocity_tolerance * velocity_tolerance && force_angle >= angle_tolerance
        {
            let mut rotation_increment = Quat::from_rotation_arc(current_heading, desired_heading);
            rotation_increment = Quat::IDENTITY.slerp(
                rotation_increment,
                (max_turn_speed / force_angle * delta_time).clamp(0.0, 1.0),
            );

            return (Vec3::ZERO, (rotation_increment * rotation));
        }
    }

    let (new_rotation, new_heading) = if let Some(desired_heading) = desired_heading {
        let force_angle = desired_heading.angle_between(current_heading);

        if force_angle >= angle_tolerance {
            let mut rotation_increment = Quat::from_rotation_arc(current_heading, desired_heading);

            rotation_increment = Quat::IDENTITY.slerp(
                rotation_increment,
                (max_turn_speed / force_angle * delta_time).clamp(0.0, 1.0),
            );

            let rotation = rotation_increment * rotation;

            (rotation, rotation.mul_vec3(Vec3::X).normalize())
        } else {
            (rotation, desired_heading)
        }
    } else {
        (rotation, current_heading)
    };

    let projected_velocity = new_heading * velocity.length();
    let projected_desired_velocity = new_heading.dot(desired_velocity) * new_heading;
    let desired_acceleration =
        (projected_desired_velocity - projected_velocity).normalize() * max_acceleration;

    let new_velocity =
        (projected_velocity + desired_acceleration * delta_time).clamp_length_max(max_speed);

    (new_velocity, new_rotation)
}
