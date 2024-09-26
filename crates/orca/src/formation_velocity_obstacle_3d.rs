use std::f32::consts::PI;

use bevy_math::{EulerRot, Mat4, Vec3};
use bevy_render::color::Color;
use geometry::{colliders::Collider, Aabb, Plane};

use crate::{Agent3D, EPSILON};

pub struct FormationVelocityObstacle3D {
    pub relative_position: Vec3,
    pub relative_velocity: Vec3,
    pub formation_collider: Collider,
    pub obstacle_collider: Collider,
    pub formation_velocity: Vec3,
    pub formation_position: Vec3,
    pub time_horizon: f32,
}

impl FormationVelocityObstacle3D {
    const MIN_T: f32 = 0.001;

    #[must_use]
    pub fn new(formation: &Agent3D, agent_other: &Agent3D, time_horizon: f32) -> Self {
        let obstacle_collider = agent_other.shape.clone();
        let formation_collider = formation.shape.clone();
        let relative_position = agent_other.position - formation.position;
        let relative_velocity = agent_other.velocity - formation.velocity;
        let formation_velocity = formation.velocity;

        Self {
            relative_position,
            relative_velocity,
            formation_collider,
            obstacle_collider,
            formation_velocity,
            formation_position: formation.position,
            time_horizon,
        }
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn orca_plane(
        &self,
        _time_step: f32,
        number_of_yaw_samples: u16,
        number_of_pitch_samples: u16,
        roll: f32,
        gizmos: &mut bevy_gizmos::gizmos::Gizmos,
    ) -> Plane {
        let collider_shape = {
            let collider = self
                .obstacle_collider
                .minkowski_sum(&self.formation_collider);

            match collider {
                Collider::Sphere(sphere) => Aabb::new(sphere.origin, Vec3::splat(sphere.radius)),
                Collider::Aabb(aabb) => aabb,
            }
        };

        for yaw_step in 0..=number_of_yaw_samples {
            let yaw = Self::lerp(
                -PI,
                PI,
                f32::from(yaw_step) / f32::from(number_of_yaw_samples),
            );

            for pitch_step in 0..=number_of_pitch_samples {
                let pitch = Self::lerp(
                    -PI / 2.0,
                    PI / 2.0,
                    f32::from(pitch_step) / f32::from(number_of_pitch_samples),
                );

                let rotation_mat = Mat4::from_euler(EulerRot::YXZ, yaw, pitch, roll);

                let front = rotation_mat.transform_point3(Vec3::Z).normalize();
                let top = rotation_mat.transform_point3(Vec3::Y).normalize();
                let left = rotation_mat.transform_point3(Vec3::X).normalize();

                let (y_side_min_t, y_side_max_t) = {
                    let position_dot_v = self.relative_position.dot(top);
                    let velocity_dot_v = self.relative_velocity.dot(top);
                    let position_dot_v_abs = position_dot_v.abs();
                    let velocity_dot_v_abs = velocity_dot_v.abs();

                    // If the velocity is zero in the direction of up/down and the position is
                    // within the top and bottom sides
                    // of the collider, then there is a permanent collision
                    if velocity_dot_v_abs < EPSILON
                        && position_dot_v_abs <= collider_shape.half_sizes.y
                    {
                        (0.0, self.time_horizon)
                    } else
                    // If the velocity is zero in the direction of up/down and the position is
                    // outside the top and bottom sides of the collider, then there can never be a
                    // collision
                    if velocity_dot_v_abs < EPSILON {
                        // will never collide
                        continue;
                    } else {
                        // If the velocity is not zero in the direction of up/down, then
                        // calculate the time of collision If the velocity is positive, then the
                        // object is moving up. If the velocity is negative, then the object is
                        // moving down. If the object is moving up, then the bottom side of the
                        // collider is the side that will collide first (allowing negative time
                        // here) If the object is moving down, then the top side of the collider
                        // is the side that will collide first (allowing negative time here)
                        let is_moving_up = velocity_dot_v > 0.0;

                        let top_t =
                            -(position_dot_v - collider_shape.half_sizes.y) / velocity_dot_v;

                        let bottom_t =
                            -(position_dot_v + collider_shape.half_sizes.y) / velocity_dot_v;

                        if is_moving_up {
                            (bottom_t, top_t)
                        } else {
                            (top_t, bottom_t)
                        }
                    }
                };

                let (x_side_min_t, x_side_max_t) = {
                    let position_dot_w = self.relative_position.dot(left);
                    let velocity_dot_w = self.relative_velocity.dot(left);
                    let velocity_dot_w_abs = velocity_dot_w.abs();
                    let position_dot_w_abs = position_dot_w.abs();

                    // If the velocity is zero in the direction of left/right and the position is within the left and right sides
                    // of the collider, then there is a permanent collision
                    if velocity_dot_w_abs < EPSILON
                        && position_dot_w_abs <= collider_shape.half_sizes.x
                    {
                        (0.0, self.time_horizon)
                    } else
                    // If the velocity is zero in the direction of left/right and the position is outside the left and right sides
                    // of the collider, then there can never be a collision
                    if velocity_dot_w_abs < EPSILON {
                        // will never collide
                        continue;
                    } else {
                        // If the velocity is not zero in the direction of left/right, then
                        // calculate the time of collision
                        // If the velocity is positive, then the object is moving to the right
                        // If the velocity is negative, then the object is moving to the left
                        // If the object is moving to the left, then the right side of the
                        // collider is the side that will collide first (allowing negative time
                        // here)
                        // If the object is moving to the right, then the left side of the
                        // collider is the side that will collide first (allowing negative time
                        // here)
                        let is_moving_left = velocity_dot_w > 0.0;

                        let left_t =
                            -(position_dot_w - collider_shape.half_sizes.x) / velocity_dot_w;
                        let right_t =
                            -(position_dot_w + collider_shape.half_sizes.x) / velocity_dot_w;

                        if is_moving_left {
                            (right_t, left_t)
                        } else {
                            (left_t, right_t)
                        }
                    }
                };

                let min_t = y_side_min_t
                    .max(x_side_min_t)
                    .clamp(Self::MIN_T, self.time_horizon.max(Self::MIN_T));

                let max_t = y_side_max_t
                    .min(x_side_max_t)
                    .clamp(Self::MIN_T, self.time_horizon.max(Self::MIN_T));

                // If the minimum time is greater than the maximum time, or the difference
                // between the two is less than epsilon, then there is no collision
                if min_t > max_t || (min_t - max_t).abs() < EPSILON {
                    continue;
                }

                let l1_0 = front.dot(self.relative_velocity)
                    + (self.relative_position.dot(front) - collider_shape.half_sizes.z) / min_t;
                let l1_1 = 2.0 * collider_shape.half_sizes.z / min_t;

                let l2_0 = front.dot(self.relative_velocity)
                    + (self.relative_position.dot(front) + collider_shape.half_sizes.z) / max_t;
                let l2_1 = 2.0 * collider_shape.half_sizes.z / max_t;

                let (t_start, t_end) = {
                    if l1_0 < l2_0 {
                        if l1_0 + l1_1 > l2_0 + l2_1 {
                            (l1_0, l1_0 + l1_1)
                        } else {
                            (l1_0, l1_0 + l2_1)
                        }
                    } else if l1_0 + l1_1 > l2_0 + l2_1 {
                        (l2_0, l2_0 + l1_1)
                    } else {
                        (l2_0, l2_0 + l2_1)
                    }
                };

                if t_end < 0.0 {
                    continue;
                }

                let t_end = t_end.clamp(0.0, 10000.0);

                gizmos.line(
                    self.formation_position + t_start * front,
                    self.formation_position + t_end * front,
                    Color::GREEN,
                );
            }
        }

        Plane::new(Vec3::ZERO, Vec3::ZERO)
    }

    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }
}
