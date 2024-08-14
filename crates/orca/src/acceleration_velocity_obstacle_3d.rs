use bevy_math::Vec3;
use geometry::{
    colliders::Collider, Circle3d, Cone, Sphere, SphereSphereIntersection, Vec3Operations,
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
    pub discrete_steps: usize,
    pub cutoff: Sphere,
    pub cutoff_secant_plane: Plane,
    pub cones: Vec<Cone>,
    pub circles: Vec<Circle3d>,
}

impl AccelerationVelocityObstacle3D {
    #[must_use]
    pub fn new(
        agent_self: &Agent3D,
        agent_other: &Agent3D,
        time_horizon: f32,
        acc_control_param: f32,
        discrete_steps: usize,
    ) -> Self {
        let shape = agent_self.shape.minkowski_sum(&agent_other.shape);
        let relative_position = agent_self.position - agent_other.position;
        let relative_velocity = agent_self.velocity - agent_other.velocity;
        let agent_velocity = agent_self.velocity;
        let total_responsibility = agent_self.responsibility + agent_other.responsibility;
        let agent_self_responsibility = agent_self.responsibility / total_responsibility;
        let bounding_radius = shape.bounding_sphere().radius;

        let (cutoff, cones, cutoff_secant_plane, circles) = {
            let mut spheres = Vec::with_capacity(discrete_steps);

            for i in 0..discrete_steps {
                #[allow(clippy::cast_precision_loss)]
                let t = Self::lerp(time_horizon, 0.001, i as f32 / discrete_steps as f32);

                let center =
                    Self::avo_center(acc_control_param, relative_velocity, relative_position, t);
                let scale_factor = Self::scale_factor(acc_control_param, t);

                spheres.push(Sphere::new(bounding_radius * scale_factor, center));
            }

            let cutoff = {
                let mut result = &spheres[0];

                for sphere in &spheres {
                    if let SphereSphereIntersection::Inside = sphere.intersect_sphere(result) {
                        result = sphere;
                    } else {
                        break;
                    }
                }

                result.clone()
            };

            let cutoff_secant_plane = cutoff.get_secant_plane(Vec3::ZERO);

            let mut circles = Vec::with_capacity(discrete_steps);

            {
                let radius = {
                    let mut max_radius = 0.0;

                    for sphere in &spheres {
                        if let Some(intersection) = sphere.intersect_plane(&cutoff_secant_plane) {
                            if intersection.radius > max_radius {
                                max_radius = intersection.radius;
                            }
                        } else {
                            break;
                        }
                    }

                    max_radius
                };

                circles.push(Circle3d::new(
                    radius,
                    cutoff_secant_plane.origin,
                    cutoff_secant_plane.normal,
                ));
            }

            for i in 0..discrete_steps - 1 {
                let normal = (spheres[i + 1].origin - spheres[i].origin).normalize();
                let origin = spheres[i].origin;
                let plane = Plane::new(origin, normal);

                let radius = {
                    let mut max_radius = spheres[i].radius;

                    for sphere in spheres.iter().skip(i + 1) {
                        if let Some(intersection) = sphere.intersect_plane(&plane) {
                            if intersection.radius > max_radius {
                                max_radius = intersection.radius;
                            }
                        } else {
                            break;
                        }
                    }

                    max_radius
                };

                circles.push(Circle3d::new(radius, origin, normal));
            }

            {
                let mut i = 0;
                loop {
                    if circles[i].radius < EPSILON {
                        circles.remove(i);
                    } else if circles[i + 1].radius - circles[i].radius < EPSILON {
                        circles.remove(i + 1);
                    } else if (circles[i + 1].origin - circles[i].origin).length_squared() < EPSILON
                    {
                        circles.remove(i + 1);
                    }

                    i += 1;
                    if i == circles.len() - 2 {
                        break;
                    }
                }
            }

            let mut cones = Vec::with_capacity(circles.len() - 1);
            for i in 0..circles.len() - 1 {
                let mut cone = Cone::new(
                    circles[i].radius,
                    circles[i].origin,
                    circles[i + 1].radius,
                    circles[i + 1].origin,
                );

                if i == circles.len() - 2 {
                    cone.max_height = None;
                }

                cones.push(cone);
            }

            (cutoff, cones, cutoff_secant_plane, circles)
        };

        Self {
            relative_position,
            relative_velocity,
            shape,
            agent_velocity,
            time_horizon,
            responsibility: agent_self_responsibility,
            acc_control_param,
            discrete_steps,
            cutoff,
            cutoff_secant_plane,
            cones,
            circles,
        }
    }

    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn orca_plane(&self, time_step: f32) -> Plane {
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
        } else {
            let (closest, normal) = self.cutoff.closest_point_and_normal(self.relative_velocity);
            let mut closest = closest;
            let mut normal = normal;
            let mut square_distance = (closest - self.relative_velocity).length_squared();

            for cone in &self.cones {
                let (p, n) = cone.closest_point_and_normal(self.relative_velocity);
                let dist_sq = (p - self.relative_velocity).length_squared();

                if dist_sq < square_distance {
                    square_distance = dist_sq;
                    closest = p;
                    normal = n;
                }
            }

            let u = closest - self.relative_velocity;

            (u, normal)
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
