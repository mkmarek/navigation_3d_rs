use bevy_math::{Vec2, Vec3};

use crate::{LineSegment2D, Vec2Operations, Vec3Operations, EPSILON};

#[derive(Debug)]
pub struct Cone {
    pub vertex: Vec3,
    pub direction: Vec3,
    pub radius: f32,
    pub min_height: Option<f32>,
    pub max_height: Option<f32>,
}

impl Cone {
    #[must_use]
    pub fn new(
        front_radius: f32,
        front_position: Vec3,
        back_radius: f32,
        back_position: Vec3,
    ) -> Self {
        let (min, min_position) = {
            if front_radius > back_radius {
                (back_radius, back_position)
            } else {
                (front_radius, front_position)
            }
        };

        let (max, max_position) = {
            if front_radius < back_radius {
                (back_radius, back_position)
            } else {
                (front_radius, front_position)
            }
        };

        let back_to_front = min_position - max_position;
        let distance = back_to_front.length();
        let direction = back_to_front / distance;

        let front_to_vertex_dist = distance * min / (max - min);
        let vertex = front_to_vertex_dist * direction + min_position;

        Self {
            vertex,
            direction: -direction,
            radius: min / front_to_vertex_dist,
            min_height: Some(front_to_vertex_dist),
            max_height: Some(distance + front_to_vertex_dist),
        }
    }

    #[must_use]
    pub fn only_min(vertex: Vec3, front_radius: f32, front_position: Vec3) -> Self {
        let direction = front_position - vertex;
        let distance = direction.length();
        let direction = direction / distance;

        Self {
            vertex,
            direction,
            radius: front_radius / distance,
            min_height: Some(distance),
            max_height: None,
        }
    }

    #[must_use]
    pub fn only_max(vertex: Vec3, back_radius: f32, back_position: Vec3) -> Self {
        let direction = back_position - vertex;
        let distance = direction.length();
        let direction = direction / distance;

        Self {
            vertex,
            direction,
            radius: back_radius / distance,
            min_height: None,
            max_height: Some(distance),
        }
    }

    #[must_use]
    pub fn infinite(vertex: Vec3, direction: Vec3, radius: f32) -> Self {
        Self {
            vertex,
            direction: direction.normalize(),
            radius: radius / direction.length(),
            min_height: Some(0.0),
            max_height: None,
        }
    }
}

impl Vec3Operations for Cone {
    fn contains(&self, pt: Vec3) -> bool {
        let relative_pt = pt - self.vertex;
        let height = relative_pt.dot(self.direction);

        if let Some(min_height) = self.min_height {
            if height < min_height {
                return false;
            }
        }

        if let Some(max_height) = self.max_height {
            if height > max_height {
                return false;
            }
        }

        let projection = self.direction * height;
        let perpendicular_component_sq = (relative_pt - projection).length_squared();

        if perpendicular_component_sq > self.radius * self.radius * height * height {
            return false;
        }

        true
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        if self.contains(pt) {
            return pt;
        }

        let (closest_point, _) = self.closest_point_and_normal(pt);

        closest_point
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let relative_pt = pt - self.vertex;
        let x_component = self.direction.dot(relative_pt);
        let projected_pt = self.direction * x_component;
        let y_component = (relative_pt - projected_pt).length_squared();
        let y_component = if y_component < EPSILON {
            0.0
        } else {
            y_component.sqrt()
        };

        let edge_1_dir = Vec2::new(1.0, self.radius).normalize();
        let edge_2_dir = Vec2::new(1.0, -self.radius).normalize();

        let min_edge = if let Some(min_height) = self.min_height {
            (min_height.powi(2) + (self.radius * min_height).powi(2)).sqrt()
        } else {
            0.0
        };

        let max_edge = if let Some(max_height) = self.max_height {
            (max_height.powi(2) + (self.radius * max_height).powi(2)).sqrt()
        } else {
            f32::INFINITY
        };

        let edge_1 = LineSegment2D::new(Vec2::ZERO, edge_1_dir, min_edge, max_edge);
        let edge_2 = LineSegment2D::new(Vec2::ZERO, edge_2_dir, min_edge, max_edge);

        let (closest, normal) = {
            let pt = Vec2::new(x_component, y_component);
            let closest_1 = edge_1.closest_point_and_normal(pt);
            let closest_2 = edge_2.closest_point_and_normal(pt);

            if (closest_1.0 - pt).length_squared() < (closest_2.0 - pt).length_squared() {
                closest_1
            } else {
                closest_2
            }
        };

        let perpendicular_direction = if y_component.abs() < EPSILON {
            let dot_x = Vec3::X.dot(self.direction);
            let dot_y = Vec3::Y.dot(self.direction);
            let dot_z = Vec3::Z.dot(self.direction);

            let basis_angle = if dot_x.abs() < dot_y.abs() && dot_x.abs() < dot_z.abs() {
                Vec3::X
            } else if dot_y.abs() < dot_z.abs() {
                Vec3::Y
            } else {
                Vec3::Z
            };

            self.direction.cross(basis_angle).normalize()
        } else {
            (relative_pt - projected_pt).normalize()
        };

        if closest.length_squared() < EPSILON {
            let point = self.vertex;
            let normal = (pt - point).normalize();

            return (point, normal);
        }

        let point = self.vertex + self.direction * closest.x + perpendicular_direction * closest.y;
        let normal = (self.direction * normal.x + perpendicular_direction * normal.y).normalize();

        if normal.dot(pt - point) < 0.0 {
            (point, -normal)
        } else {
            (point, normal)
        }
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let (closest_point, normal) = self.closest_point_and_normal(pt);

        if self.contains(pt) {
            (pt - closest_point).dot(normal)
        } else {
            (closest_point - pt).length()
        }
    }
}
