use std::{collections::HashMap, f32::consts::PI};

use bevy_math::{EulerRot, Mat4, Vec3};
use geometry::{colliders::Collider, Aabb, Plane, Triangle, Vec3Operations};

use crate::{Agent3D, EPSILON};

pub struct FormationVelocityObstacle3D {
    relative_position: Vec3,
    obstacle_velocity: Vec3,
    formation_collider: Collider,
    obstacle_collider: Collider,
    formation_velocity: Vec3,
    time_horizon: f32,
}

impl FormationVelocityObstacle3D {
    const MIN_T: f32 = 0.001;

    #[must_use]
    pub fn new(formation: &Agent3D, agent_other: &Agent3D, time_horizon: f32) -> Self {
        let obstacle_collider = agent_other.shape.clone();
        let formation_collider = formation.shape.clone();
        let relative_position = agent_other.position - formation.position;
        let obstacle_velocity = agent_other.velocity;
        let formation_velocity = formation.velocity;

        Self {
            relative_position,
            obstacle_velocity,
            formation_collider,
            obstacle_collider,
            formation_velocity,
            time_horizon,
        }
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn orca_plane(
        &self,
        number_of_yaw_samples: u16,
        number_of_pitch_samples: u16,
        roll: f32,
    ) -> Option<Plane> {
        let collider_shape = {
            let collider = self
                .obstacle_collider
                .minkowski_sum(&self.formation_collider);

            match collider {
                Collider::Sphere(sphere) => Aabb::new(sphere.origin, Vec3::splat(sphere.radius)),
                Collider::Aabb(aabb) => aabb,
            }
        };

        // If there is collision, we project on the boundary of the collider
        let moved_shape = Aabb::new(self.relative_position, collider_shape.half_sizes);
        if moved_shape.contains(Vec3::ZERO) {
            let (pt, normal) = moved_shape.closest_point_and_normal(Vec3::ZERO);
            return Some(Plane::new(pt, normal));
        }

        let triangles =
            self.construct_vo_mesh(number_of_yaw_samples, number_of_pitch_samples, roll);

        let mut min_distance = f32::MAX;
        let mut point = Vec3::ZERO;
        let mut normal = Vec3::ZERO;

        for triangle in triangles {
            let (pt, n) = triangle.closest_point_and_normal(self.formation_velocity);
            let distance = (pt - self.formation_velocity).length_squared();

            if distance + EPSILON < min_distance {
                min_distance = distance;
                point = pt;
                normal = n;
            }
        }

        if min_distance == f32::MAX {
            return None;
        }

        Some(Plane::new(point, normal))
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn construct_vo_mesh(
        &self,
        number_of_yaw_samples: u16,
        number_of_pitch_samples: u16,
        roll: f32,
    ) -> Vec<Triangle> {
        let points = self.sample_points(number_of_yaw_samples, number_of_pitch_samples, roll);

        let mut triangles = Vec::new();
        let mut points_to_process = points.keys().collect::<Vec<_>>();

        while let Some((yaw_step, pitch_step)) = points_to_process.pop() {
            let (start, end) = points[&(*yaw_step, *pitch_step)];

            let top_neighbor = points.get(&(*yaw_step, (pitch_step + 1) % number_of_pitch_samples));
            let right_neighbor = points.get(&((yaw_step + 1) % number_of_yaw_samples, *pitch_step));
            let top_right_neighbor = points.get(&(
                (yaw_step + 1) % number_of_yaw_samples,
                (pitch_step + 1) % number_of_pitch_samples,
            ));

            match (top_neighbor, right_neighbor, top_right_neighbor) {
                (None, None, None) => {}
                (None, None, Some(_top_right)) => {}
                (None, Some(right), None) => {
                    let (right_start, right_end) = right;

                    triangles.push(Triangle::new([start, *right_start, end]));
                    triangles.push(Triangle::new([*right_start, *right_end, end]));
                }
                (None, Some(right), Some(top_right)) => {
                    let (right_start, right_end) = right;
                    let (top_right_start, top_right_end) = top_right;

                    // here it is

                    triangles.push(Triangle::new([start, *right_start, *top_right_start]));
                    triangles.push(Triangle::new([end, *top_right_end, *right_end]));
                    triangles.push(Triangle::new([start, *top_right_start, end]));
                    triangles.push(Triangle::new([*top_right_start, *top_right_end, end]));
                }
                (Some(top), None, None) => {
                    let (top_start, top_end) = top;

                    triangles.push(Triangle::new([start, end, *top_start]));
                    triangles.push(Triangle::new([*top_start, end, *top_end]));
                }
                (Some(top), None, Some(top_right)) => {
                    let (top_start, top_end) = top;
                    let (top_right_start, top_right_end) = top_right;

                    triangles.push(Triangle::new([start, *top_right_start, *top_start]));
                    triangles.push(Triangle::new([end, *top_end, *top_right_end]));
                    triangles.push(Triangle::new([start, end, *top_right_start]));
                    triangles.push(Triangle::new([*top_right_start, end, *top_right_end]));
                }
                (Some(top), Some(right), None) => {
                    let (top_start, top_end) = top;
                    let (right_start, right_end) = right;

                    triangles.push(Triangle::new([start, *right_start, *top_start]));
                    triangles.push(Triangle::new([end, *top_end, *right_end]));
                    triangles.push(Triangle::new([*top_start, *right_start, *right_end]));
                    triangles.push(Triangle::new([*top_start, *right_end, *top_end]));
                }
                (Some(top), Some(right), Some(top_right)) => {
                    let (top_start, top_end) = top;
                    let (right_start, right_end) = right;
                    let (top_right_start, top_right_end) = top_right;

                    triangles.push(Triangle::new([start, *right_start, *top_start]));
                    triangles.push(Triangle::new([end, *top_end, *right_end]));
                    triangles.push(Triangle::new([*top_start, *right_start, *top_right_start]));
                    triangles.push(Triangle::new([*top_end, *top_right_end, *right_end]));
                }
            }

            // a case where on the left side there are no points so that side won't be evaluated
            // and we'll have some missing triangles
            let left_neighbor = points.get(&(
                (yaw_step + number_of_yaw_samples - 1) % number_of_yaw_samples,
                *pitch_step,
            ));
            let left_top_neighbor = points.get(&(
                (yaw_step + number_of_yaw_samples - 1) % number_of_yaw_samples,
                (pitch_step + 1) % number_of_pitch_samples,
            ));

            if left_neighbor.is_none() && left_top_neighbor.is_none() {
                if let Some((top_start, top_end)) = top_neighbor {
                    triangles.push(Triangle::new([start, *top_start, end]));
                    triangles.push(Triangle::new([*top_start, *top_end, end]));
                }
            }
            if left_neighbor.is_none() {
                if let (Some((top_start, top_end)), Some((left_top_start, left_top_end))) =
                    (top_neighbor, left_top_neighbor)
                {
                    triangles.push(Triangle::new([start, *top_start, *left_top_start]));
                    triangles.push(Triangle::new([end, *left_top_end, *top_end]));
                    triangles.push(Triangle::new([start, *left_top_start, end]));
                    triangles.push(Triangle::new([*left_top_start, *left_top_end, end]));
                }
            }

            let bottom_neighbor = points.contains_key(&(
                *yaw_step,
                (pitch_step + number_of_pitch_samples - 1) % number_of_pitch_samples,
            ));
            let bottom_right_neighbor = points.contains_key(&(
                (yaw_step + 1) % number_of_yaw_samples,
                (pitch_step + number_of_pitch_samples - 1) % number_of_pitch_samples,
            ));

            if !bottom_neighbor && !bottom_right_neighbor {
                if let Some((right_start, right_end)) = right_neighbor {
                    triangles.push(Triangle::new([start, end, *right_start]));
                    triangles.push(Triangle::new([*right_start, end, *right_end]));
                }
            }
        }

        triangles
    }

    #[allow(clippy::too_many_lines)]
    fn sample_points(
        &self,
        number_of_yaw_samples: u16,
        number_of_pitch_samples: u16,
        roll: f32,
    ) -> HashMap<(u16, u16), (Vec3, Vec3)> {
        let collider_shape = {
            let collider = self
                .obstacle_collider
                .minkowski_sum(&self.formation_collider);

            match collider {
                Collider::Sphere(sphere) => Aabb::new(sphere.origin, Vec3::splat(sphere.radius)),
                Collider::Aabb(aabb) => aabb,
            }
        };

        let mut points = HashMap::new();

        let mut most_min_t = f32::MAX;
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
                    let velocity_dot_v = self.obstacle_velocity.dot(top);
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
                    let velocity_dot_w = self.obstacle_velocity.dot(left);
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

                let l1_0 = front.dot(self.obstacle_velocity)
                    + (self.relative_position.dot(front) - collider_shape.half_sizes.z) / min_t;
                let l1_1 = 2.0 * collider_shape.half_sizes.z / min_t;

                let l2_0 = front.dot(self.obstacle_velocity)
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

                let t_end = 10000.0;
                let t_start = t_start.clamp(0.0, t_end);

                most_min_t = most_min_t.min(t_start);

                points.insert((yaw_step, pitch_step), (t_start * front, t_end * front));
            }
        }

        points
    }

    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }
}
