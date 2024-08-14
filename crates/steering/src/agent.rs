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

    let current_heading = rotation.mul_vec3(Vec3::X).normalize();
    let max_acceleration = max_force / mass;

    // The velocity tolerance is how much the velocity can change in 2 frames
    let velocity_tolerance = (max_acceleration * delta_time * 2.0).max(0.001);

    // The angle tolerance is how much the angle can change in 2 frames
    let angle_tolerance = (max_turn_speed * delta_time * 2.0).min(0.001);

    let desired_heading = if desired_velocity.length_squared() > f32::EPSILON {
        Some(desired_velocity.normalize())
    } else {
        None
    };

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

    //let velocity_diff = (desired_velocity - velocity).normalize_or_zero() * max_acceleration;
    //let new_velocity = (velocity + velocity_diff * delta_time).clamp_length_max(max_speed);

    let projected_velocity = new_heading * velocity.length();
    let projected_desired_velocity = new_heading.dot(desired_velocity) * new_heading;
    let velocity_diff = projected_desired_velocity - projected_velocity;

    let new_velocity = if velocity_diff.length_squared() <= velocity_tolerance * velocity_tolerance
    {
        projected_desired_velocity
    } else {
        let desired_acceleration = velocity_diff.normalize() * max_acceleration;
        (projected_velocity + desired_acceleration * delta_time).clamp_length_max(max_speed)
    };

    let lateral_velocity_diff = desired_velocity - new_velocity;
    let lateral_velocity =
        lateral_velocity_diff.clamp_length_max(max_acceleration * delta_time / 2.0);

    let new_velocity = (new_velocity + lateral_velocity).clamp_length_max(max_speed);

    (new_velocity, new_rotation)
}
