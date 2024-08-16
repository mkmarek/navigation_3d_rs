use bevy_gizmos::gizmos::Gizmos;
use bevy_math::{Mat2, Vec2, Vec3};
use bevy_render::color::Color;
use geometry::{
    colliders::Collider, Arc2D, LineSegment2D, LineSegment2DIntersection,
    LineSegment2DIntersectionResult, Ray2DIntersection, Ray2DIntersectionResult, Sphere,
    Vec2Operations, Vec3Operations,
};

use crate::{Agent3D, Plane, EPSILON};

pub struct AccelerationVelocityObstacle3D {
    pub relative_position: Vec3,
    pub relative_velocity: Vec3,
    pub shape: Collider,
    pub agent_velocity: Vec3,
    pub time_horizon: f32,
    pub acc_control_param: f32,
    pub responsibility: f32,
    pub discrete_steps: u16,
}

#[derive(Debug)]
enum AVOBoundary {
    LineSegment(LineSegment2D),
    Arc(Arc2D),
}

impl AVOBoundary {
    pub fn new(
        v_ab: Vec2,
        p_ab: Vec2,
        radius: f32,
        time_horizon: f32,
        acc_control_param: f32,
        discrete_steps: u16,
    ) -> Vec<AVOBoundary> {
        let mut left_boundary = Vec::with_capacity(discrete_steps as usize);
        let mut right_boundary = Vec::with_capacity(discrete_steps as usize);

        for i in 0..discrete_steps {
            let t1 = Self::lerp(
                0.001,
                time_horizon,
                f32::from(i) / f32::from(discrete_steps),
            );
            let t2 = Self::lerp(
                0.001,
                time_horizon,
                f32::from(i + 1) / f32::from(discrete_steps),
            );

            let p1 = Self::boundary(t1, acc_control_param, radius, v_ab, p_ab, 1.0);
            let p2 = Self::boundary(t2, acc_control_param, radius, v_ab, p_ab, 1.0);

            if !p1.is_nan() && !p2.is_nan() && p1.distance_squared(p2) > EPSILON {
                let line_segment = LineSegment2D::from_two_points(p1, p2);
                left_boundary.push(line_segment);
            }

            let p1 = Self::boundary(t1, acc_control_param, radius, v_ab, p_ab, -1.0);
            let p2 = Self::boundary(t2, acc_control_param, radius, v_ab, p_ab, -1.0);

            if !p1.is_nan() && !p2.is_nan() && p1.distance_squared(p2) > EPSILON {
                let line_segment = LineSegment2D::from_two_points(p2, p1);
                right_boundary.push(line_segment);
            }
        }

        Self::clean_self_intersections(&mut left_boundary);
        Self::clean_self_intersections(&mut right_boundary);

        let arc = {
            let boundary_a = left_boundary.last().unwrap();
            let boundary_b = right_boundary.last().unwrap();

            let a = boundary_a.end();
            let b = boundary_b.origin;

            if let Ray2DIntersectionResult::Point(t) =
                boundary_a.to_ray().intersect(&boundary_b.to_ray())
            {
                let r = radius * Self::scale_factor(acc_control_param, time_horizon);

                if r < a.distance(b) / 2.0 {
                    AVOBoundary::LineSegment(LineSegment2D::from_two_points(a, b))
                } else {
                    let (a1, a2) = Arc2D::from_points(r, a, b);
                    let intersection = boundary_a.origin + boundary_a.direction * t;

                    if a1.center.distance_squared(intersection)
                        > a2.center.distance_squared(intersection)
                    {
                        AVOBoundary::Arc(a1)
                    } else {
                        AVOBoundary::Arc(a2)
                    }
                }
            } else {
                AVOBoundary::LineSegment(LineSegment2D::from_two_points(a, b))
            }
        };

        let mut result = Vec::with_capacity(discrete_steps as usize * 2 + 1);

        left_boundary
            .drain(..)
            .for_each(|line_segment| result.push(AVOBoundary::LineSegment(line_segment)));
        right_boundary.reverse();

        result.push(arc);

        right_boundary
            .drain(..)
            .for_each(|line_segment| result.push(AVOBoundary::LineSegment(line_segment)));

        result
    }

    fn clean_self_intersections(boundary: &mut Vec<LineSegment2D>) {
        for i in 0..boundary.len() {
            let mut intesecting_line = None;
            let mut t = None;

            for j in i + 1..boundary.len() {
                let intersection = boundary[i].intersect(&boundary[j]);
                if let LineSegment2DIntersectionResult::Point(t1) = intersection {
                    if (t1 - boundary[i].t_max).abs() < EPSILON
                        || (t1 - boundary[j].t_min).abs() < EPSILON
                    {
                        continue;
                    }

                    intesecting_line = Some(j);
                    t = Some(t1);
                }
            }

            if let Some(intesecting_line) = intesecting_line {
                boundary[i].t_max = t.unwrap();
                boundary.drain(i + 1..intesecting_line);
            }
        }
    }

    fn scale_factor(acc_control_param: f32, t: f32) -> f32 {
        let param = acc_control_param * ((-t / acc_control_param).exp() - 1.0);

        (t + param).recip()
    }

    fn boundary(t: f32, delta: f32, r_ab: f32, v_ab: Vec2, p_ab: Vec2, sign: f32) -> Vec2 {
        // Precalcualte some parameters that are used multiple times
        let exp = (-t / delta).exp();
        let param = exp - 1.0;
        let delta_param = delta * param;

        // Calculate the center value
        let center = (v_ab * delta_param - p_ab) / (delta_param + t);

        // Calculate the radius value
        let radius = r_ab / (delta_param + t);

        // Calculate the derivative of the center value
        let center_dot = ((v_ab * delta_param - p_ab) * param) / (delta_param + t).powi(2)
            - (v_ab * exp) / (delta_param + t);

        // Calculate the derivative of the radius value
        let radius_dot = r_ab * param / (delta_param + t).powi(2);

        // Precompute he radius rate of change
        let rrc = radius / radius_dot * center_dot;

        // Calculat the tangent result
        let tangent = {
            let p_len_sq = rrc.length_squared();
            let r = radius;

            let l = (p_len_sq - r * r).sqrt();
            let mat = Mat2::from_cols(Vec2::new(l, sign * r), Vec2::new(-sign * r, l));

            mat * rrc * (l / p_len_sq)
        };

        // Calculate the boundary
        center - rrc + tangent
    }

    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }
}

impl Vec2Operations for AVOBoundary {
    fn contains(&self, pt: Vec2) -> bool {
        match self {
            AVOBoundary::LineSegment(line_segment) => line_segment.contains(pt),
            AVOBoundary::Arc(arc) => arc.contains(pt),
        }
    }

    fn constrain(&self, pt: Vec2) -> Vec2 {
        match self {
            AVOBoundary::LineSegment(line_segment) => line_segment.constrain(pt),
            AVOBoundary::Arc(arc) => arc.constrain(pt),
        }
    }

    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2) {
        match self {
            AVOBoundary::LineSegment(line_segment) => {
                let (pt, _) = line_segment.closest_point_and_normal(pt);

                // Here we need to switch the normal if its aiming below the border line
                let perp = line_segment.direction.perp();

                (pt, -perp)
            }
            AVOBoundary::Arc(arc) => arc.closest_point_and_normal(pt),
        }
    }

    fn signed_distance(&self, pt: Vec2) -> f32 {
        match self {
            AVOBoundary::LineSegment(line_segment) => line_segment.signed_distance(pt),
            AVOBoundary::Arc(arc) => arc.signed_distance(pt),
        }
    }
}

impl AccelerationVelocityObstacle3D {
    #[must_use]
    pub fn new(
        agent_self: &Agent3D,
        agent_other: &Agent3D,
        time_horizon: f32,
        acc_control_param: f32,
        discrete_steps: u16,
    ) -> Self {
        let shape = agent_self.shape.minkowski_sum(&agent_other.shape);
        let relative_position = agent_self.position - agent_other.position;
        let relative_velocity = agent_self.velocity - agent_other.velocity;
        let agent_velocity = agent_self.velocity;
        let total_responsibility = agent_self.responsibility + agent_other.responsibility;
        let agent_self_responsibility = agent_self.responsibility / total_responsibility;

        Self {
            relative_position,
            relative_velocity,
            shape,
            agent_velocity,
            time_horizon,
            responsibility: agent_self_responsibility,
            acc_control_param,
            discrete_steps,
        }
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn orca_plane(&self, time_step: f32) -> Plane {
        let radius = self.shape.bounding_sphere().radius;

        // Collision
        let (u, normal) = if self.shape.contains(self.relative_position) {
            // project on a cutoff plane at time_step
            let cutoff_center = Self::avo_center(
                self.acc_control_param,
                self.relative_velocity,
                self.relative_position,
                time_step,
            );

            let cutoff_scale_factor = Self::scale_factor(self.acc_control_param, time_step);

            let time_step_cutoff_shape = self.shape.scale(cutoff_scale_factor);
            let from_cutoff_center_to_relative_velocity = self.relative_velocity - cutoff_center;

            let (p, normal) = time_step_cutoff_shape
                .closest_point_and_normal(from_cutoff_center_to_relative_velocity);

            // p is relative to cutoff center, we need to make it relative to relative_velocity
            let u = p + cutoff_center - self.relative_velocity;

            (u, normal)
        } else if self.relative_velocity.length_squared() < EPSILON {
            let cutoff_sphere = {
                let cutoff_center = Self::avo_center(
                    self.acc_control_param,
                    self.relative_velocity,
                    self.relative_position,
                    self.time_horizon,
                );

                let cutoff_radius =
                    radius * Self::scale_factor(self.acc_control_param, self.time_horizon);

                Sphere::new(cutoff_radius, cutoff_center)
            };

            let (p, normal) = cutoff_sphere.closest_point_and_normal(Vec3::ZERO);
            let u = p - self.relative_velocity;

            (u, normal)
        } else {
            let p0 = Vec3::ZERO;
            let p1 = self.relative_velocity;
            let p2 = {
                if self
                    .relative_position
                    .normalize_or_zero()
                    .cross(p1.normalize_or_zero())
                    .length_squared()
                    < EPSILON
                {
                    let p1_dot_x = p1.dot(Vec3::X);
                    let p1_dot_y = p1.dot(Vec3::Y);
                    let p1_dot_z = p1.dot(Vec3::Z);

                    let basis =
                        if p1_dot_x.abs() < p1_dot_y.abs() && p1_dot_x.abs() < p1_dot_z.abs() {
                            Vec3::X
                        } else if p1_dot_y.abs() < p1_dot_z.abs() {
                            Vec3::Y
                        } else {
                            Vec3::Z
                        };

                    p1.cross(basis).normalize_or_zero()
                } else {
                    self.relative_position
                }
            };

            let plane = Plane::from_points(p0, p1, p2);
            let v_ab = plane.project_2d(self.relative_velocity);
            let p_ab = plane.project_2d(self.relative_position);

            let boundary = AVOBoundary::new(
                v_ab,
                p_ab,
                radius,
                self.time_horizon,
                self.acc_control_param,
                self.discrete_steps,
            );

            let (mut u, mut normal) = boundary[0].closest_point_and_normal(v_ab);

            for (i, boundary) in boundary.iter().enumerate().skip(1) {
                let (p, n) = boundary.closest_point_and_normal(v_ab);

                if (p - v_ab).length_squared() < (u - v_ab).length_squared() {
                    u = p;
                    normal = n;
                }
            }

            let u = plane.project_3d(u);
            let normal = plane.project_3d(normal);

            (u - self.relative_velocity, normal)
        };

        Plane::new(self.agent_velocity + self.responsibility * u, normal)
    }

    fn avo_center(
        acc_control_param: f32,
        relative_velocity: Vec3,
        relative_position: Vec3,
        t: f32,
    ) -> Vec3 {
        let param = acc_control_param * ((-t / acc_control_param).exp() - 1.0);

        (param * relative_velocity - relative_position) / (t + param)
    }

    fn scale_factor(acc_control_param: f32, t: f32) -> f32 {
        let param = acc_control_param * ((-t / acc_control_param).exp() - 1.0);

        (t + param).recip()
    }
}
