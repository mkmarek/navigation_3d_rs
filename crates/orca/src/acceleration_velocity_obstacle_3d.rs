use bevy_gizmos::gizmos::Gizmos;
use bevy_math::{Mat2, Vec2, Vec3};
use bevy_render::color::Color;
use geometry::{
    colliders::Collider, Arc2D, Cone, LineSegment2D, LineSegment2DIntersection,
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
            if let (Some(boundary_a), Some(boundary_b)) =
                (left_boundary.last(), right_boundary.last())
            {
                let a = boundary_a.end();
                let b = boundary_b.origin;

                if let Ray2DIntersectionResult::Point(t) =
                    boundary_a.to_ray().intersect(&boundary_b.to_ray())
                {
                    let r = radius * Self::scale_factor(acc_control_param, time_horizon);

                    if r < a.distance(b) / 2.0 {
                        Some(AVOBoundary::LineSegment(LineSegment2D::from_two_points(
                            a, b,
                        )))
                    } else {
                        let (a1, a2) = Arc2D::from_points(r, a, b);
                        let intersection = boundary_a.origin + boundary_a.direction * t;

                        if a1.center.distance_squared(intersection)
                            > a2.center.distance_squared(intersection)
                        {
                            Some(AVOBoundary::Arc(a1))
                        } else {
                            Some(AVOBoundary::Arc(a2))
                        }
                    }
                } else {
                    Some(AVOBoundary::LineSegment(LineSegment2D::from_two_points(
                        a, b,
                    )))
                }
            } else {
                None
            }
        };

        let mut result = Vec::with_capacity(discrete_steps as usize * 2 + 1);

        left_boundary
            .drain(..)
            .for_each(|line_segment| result.push(AVOBoundary::LineSegment(line_segment)));
        right_boundary.reverse();

        if let Some(arc) = arc {
            result.push(arc);
        }

        right_boundary
            .drain(..)
            .for_each(|line_segment| result.push(AVOBoundary::LineSegment(line_segment)));

        result
    }

    fn clean_self_intersections(boundary: &mut Vec<LineSegment2D>) {
        for i in 0..boundary.len() {
            let mut intesecting_line_and_t = None;

            for j in i + 1..boundary.len() {
                let intersection = boundary[i].intersect(&boundary[j]);
                if let LineSegment2DIntersectionResult::Point(t1) = intersection {
                    if (t1 - boundary[i].t_max).abs() < EPSILON
                        || (t1 - boundary[j].t_min).abs() < EPSILON
                    {
                        continue;
                    }

                    intesecting_line_and_t = Some((j, t1));
                }
            }

            if let Some((intesecting_line, t)) = intesecting_line_and_t {
                boundary[i].t_max = t;
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
    pub fn orca_plane(&self, time_step: f32) -> Option<Plane> {
        let radius = self.shape.bounding_sphere().radius;
        let shape_sphere = Sphere::new(radius, Vec3::ZERO);

        //let cutoff_ct = Self::avo_center(
        //    self.acc_control_param,
        //    self.relative_velocity,
        //    self.relative_position,
        //    self.time_horizon,
        //);

        //let cutoff_radius = radius * Self::scale_factor(self.acc_control_param, self.time_horizon);
        //gizmos.sphere(
        //    cutoff_ct - self.relative_velocity + offset + self.agent_velocity,
        //    Quat::IDENTITY,
        //    cutoff_radius,
        //    Color::GREEN,
        //);
        // Collision
        let (u, normal) = if shape_sphere.contains(self.relative_position) {
            // project on a cutoff plane at time_step
            let time_step_ct = Self::avo_center(
                self.acc_control_param,
                self.relative_velocity,
                self.relative_position,
                time_step,
            );

            let cutoff_ct = Self::avo_center(
                self.acc_control_param,
                self.relative_velocity,
                self.relative_position,
                self.time_horizon,
            );

            let time_step_radius = radius * Self::scale_factor(self.acc_control_param, time_step);
            let cutoff_radius =
                radius * Self::scale_factor(self.acc_control_param, self.time_horizon);

            let direction_from_cutoff_to_time_step = (time_step_ct - cutoff_ct).normalize_or_zero();
            let direction_from_relative_velocity_to_cutoff =
                (cutoff_ct - self.relative_velocity).normalize_or_zero();

            //gizmos.sphere(
            //    cutoff_ct - self.relative_velocity + offset + self.agent_velocity,
            //    Quat::IDENTITY,
            //    cutoff_radius,
            //    Color::GREEN,
            //);

            //gizmos.sphere(
            //    time_step_ct - self.relative_velocity + offset + self.agent_velocity,
            //    Quat::IDENTITY,
            //    time_step_radius,
            //    Color::WHITE,
            //);

            //draw_truncated_cone(
            //    gizmos,
            //    cutoff_radius,
            //    cutoff_ct - self.relative_velocity + offset + self.agent_velocity,
            //    time_step_radius,
            //    time_step_ct - self.relative_velocity + offset + self.agent_velocity,
            //);

            let dt_sphere = Sphere::new(time_step_radius, time_step_ct);
            let cutoff_sphere = Sphere::new(cutoff_radius, cutoff_ct);

            if cutoff_sphere.is_inside(&dt_sphere) {
                let (p, normal) = dt_sphere.closest_point_and_normal(self.relative_velocity);

                (p - self.relative_velocity, normal)
            } else if direction_from_cutoff_to_time_step
                .dot(direction_from_relative_velocity_to_cutoff)
                > 0.0
            {
                let cone = Cone::new(cutoff_radius, cutoff_ct, time_step_radius, time_step_ct);
                let (p, normal) = cone.closest_point_and_normal(self.relative_velocity);

                (p - self.relative_velocity, normal)
            } else {
                let cutoff = Sphere::new(cutoff_radius, cutoff_ct);
                let (p, normal) = cutoff.closest_point_and_normal(self.relative_velocity);
                (p - self.relative_velocity, normal)
            }
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
                    let p1_dot_x = p1.normalize().dot(Vec3::X);
                    let p1_dot_y = p1.normalize().dot(Vec3::Y);
                    let p1_dot_z = p1.normalize().dot(Vec3::Z);

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

            //for boundary in &boundary {
            //    match boundary {
            //        AVOBoundary::LineSegment(line_segment) => {
            //            let from =
            //                line_segment.origin + line_segment.direction * line_segment.t_min;
            //            let to = line_segment.origin + line_segment.direction * line_segment.t_max;

            //            let from_3d = plane.project_3d(from);
            //            let to_3d = plane.project_3d(to);

            //            gizmos.line(from_3d, to_3d, Color::RED);
            //        }
            //        AVOBoundary::Arc(arc) => {
            //            for i in 0..10_u16 {
            //                let t1 = arc.point_at(f32::from(i) / 10.0);
            //                let t2 = arc.point_at(f32::from(i + 1) / 10.0);

            //                let t1_3d = plane.project_3d(t1);
            //                let t2_3d = plane.project_3d(t2);

            //                gizmos.line(t1_3d, t2_3d, Color::RED);
            //            }
            //        }
            //    }
            //}

            if boundary.is_empty() {
                return None;
            }

            let (mut u, mut normal) = boundary[0].closest_point_and_normal(v_ab);

            for boundary in boundary.iter().skip(1) {
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

        Some(Plane::new(self.responsibility * u, normal))
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

#[allow(dead_code)]
fn draw_truncated_cone(
    gizmos: &mut Gizmos,
    front_radius: f32,
    front_position: Vec3,
    back_radius: f32,
    back_position: Vec3,
) {
    const SEGMENTS: u16 = 32;

    let direction = (back_position - front_position).normalize();

    // Draw front base
    gizmos.circle(
        front_position,
        direction.normalize(),
        front_radius,
        Color::RED,
    );

    // Draw back base
    gizmos.circle(
        back_position,
        direction.normalize(),
        back_radius,
        Color::RED,
    );

    let dot_x = Vec3::X.dot(direction.normalize());
    let dot_y = Vec3::Y.dot(direction.normalize());
    let dot_z = Vec3::Z.dot(direction.normalize());

    let basis_angle = if dot_x.abs() < dot_y.abs() && dot_x.abs() < dot_z.abs() {
        Vec3::X
    } else if dot_y.abs() < dot_z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };

    // Draw sides
    for i in 0..SEGMENTS {
        let angle = f32::from(i) / f32::from(SEGMENTS) * std::f32::consts::TAU;
        let perp_1 = direction.cross(basis_angle).normalize();
        let perp_2 = direction.cross(perp_1).normalize();

        let start = front_position
            + direction
            + (angle.cos() * front_radius * perp_1)
            + (angle.sin() * front_radius * perp_2);
        let end = back_position
            + direction
            + (angle.cos() * back_radius * perp_1)
            + (angle.sin() * back_radius * perp_2);

        gizmos.line(start, end, Color::RED);
    }
}
