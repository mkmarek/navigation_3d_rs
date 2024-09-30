use std::ops::Index;

use bevy_math::{Vec2, Vec3};

use crate::Plane;

pub struct Triangle {
    points: [Vec3; 3],
    plane: Plane,
}

impl Triangle {
    pub fn new(points: [Vec3; 3]) -> Self {
        Self {
            points,
            plane: Plane::from_points(points[0], points[1], points[2]),
        }
    }

    pub fn uv(&self) -> [Vec2; 3] {
        [
            self.plane.project_2d(self.points[0]),
            self.plane.project_2d(self.points[1]),
            self.plane.project_2d(self.points[2]),
        ]
    }

    pub fn points(&self) -> &[Vec3; 3] {
        &self.points
    }

    pub fn normal(&self) -> Vec3 {
        self.plane.normal
    }
}

impl Index<usize> for Triangle {
    type Output = Vec3;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}
