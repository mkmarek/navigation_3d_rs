use std::ops::Deref;

use bevy_math::{Vec2, Vec3};
use geometry::{Circle, LineSegment2D, Plane, SecondTangentPointResult};

// When an agent wants to turn, we can calculate the plane at which the turn will happen.
// This plane will be aligned with the agent's current position, the agent's current velocity, and the new direction.
pub struct TurnPlane {
    plane: Plane,
    direction_after_turn: Vec2,
    current_direction: Vec2,
    length_to_turn_point: f32,
}

impl TurnPlane {
    #[must_use]
    pub fn new(agent_position: Vec3, turn_point: Vec3, end_point: Vec3) -> Self {
        let plane = Plane::from_points(agent_position, turn_point, end_point);

        let length_to_turn_point = (turn_point - agent_position).length();
        let turn_point_2d = plane.project_2d(turn_point);
        let end_point_2d = plane.project_2d(end_point);

        let current_direction = turn_point_2d.normalize();
        let direction_after_turn = (end_point_2d - turn_point_2d).normalize();

        Self {
            plane,
            direction_after_turn,
            current_direction,
            length_to_turn_point,
        }
    }

    pub fn turn_circle(&self, current_velocity: f32, max_turn_speed: f32) -> Circle {
        let mut perpendicular = self.current_direction.perp();
        if self.direction_after_turn.dot(perpendicular) < 0.0 {
            perpendicular = -perpendicular;
        }

        let radius = current_velocity / max_turn_speed;
        let origin = Vec2::ZERO + perpendicular * radius;

        Circle::new(radius, origin)
    }

    pub fn find_tangent_between_agent_and_turn_point(
        &self,
        turn_circle: &Circle,
    ) -> SecondTangentPointResult {
        let segment = LineSegment2D::new(
            Vec2::ZERO,
            self.current_direction,
            0.0,
            self.length_to_turn_point,
        );

        turn_circle.find_tangent_on_line_segment(&segment, self.direction_after_turn)
    }
}

impl Deref for TurnPlane {
    type Target = Plane;

    fn deref(&self) -> &Self::Target {
        &self.plane
    }
}
