use bevy_math::Vec3;

use geometry::{HalfPlane, Plane, PlaneIntersecion, Vec3Operations};

use crate::solver_2d::{incremental_optimization_2d, MaximumVelocityShape2D, OptimizationResult2D};

#[derive(Debug)]
pub enum OptimizationResult3D {
    Feasible {
        optimal_velocity: Vec3,
    },
    #[allow(dead_code)]
    Infeasible {
        last_optimal_velocity: Vec3,
    },
}

pub trait MaximumVelocityShape3D {
    fn constrain(&self, velocity: Vec3) -> Vec3;
    fn project_on_plane(&self, plane: &Plane) -> Option<impl MaximumVelocityShape2D>;
}

impl<T> MaximumVelocityShape3D for T
where
    T: Vec3Operations + PlaneIntersecion,
{
    fn constrain(&self, velocity: Vec3) -> Vec3 {
        self.constrain(velocity)
    }

    fn project_on_plane(&self, plane: &Plane) -> Option<impl MaximumVelocityShape2D> {
        self.intersect(plane)
    }
}

pub fn incremental_optimization_3d(
    preffered_velocity: Vec3,
    bounding_shape: &impl MaximumVelocityShape3D,
    planes: &[Plane],
) -> OptimizationResult3D {
    let mut optimal_velocity = bounding_shape.constrain(preffered_velocity);
    for i in 0..planes.len() {
        let plane = &planes[i];

        // Check if the velocity is inside the the zone of the plane
        // If it is, we can skip this plane
        if plane.contains(optimal_velocity) {
            continue;
        }

        // The new optimal_velocity will lie somewhere on the plane
        // first we find a intersecting 2d shape between the plane and the bounding shape
        // then we calculate intersections of all the previous planes with the current one
        // which will yield an array of half-planes. We use all of that to solve a 2d optimization
        // problem, which will give us the optimal velocity on the plane
        let mut half_planes = Vec::new();
        let bounding_shape_2d = bounding_shape.project_on_plane(plane);

        if bounding_shape_2d.is_none() {
            return OptimizationResult3D::Infeasible {
                last_optimal_velocity: optimal_velocity,
            };
        }

        let bounding_shape_2d = bounding_shape_2d.unwrap();

        let (optimal_velocity_on_plane, _) = plane.closest_point_and_normal(optimal_velocity);
        let optimal_velocity_on_plane = plane.project_2d(optimal_velocity_on_plane);

        for plane_j in planes.iter().take(i) {
            if let Some(half_plane) = HalfPlane::from_plane_intersection(plane, plane_j) {
                half_planes.push(half_plane);
            }
        }

        let result = incremental_optimization_2d(
            optimal_velocity_on_plane,
            &bounding_shape_2d,
            &half_planes,
        );

        if let OptimizationResult2D::Feasible {
            optimal_velocity: opt,
        } = result
        {
            optimal_velocity = plane.project_3d(opt);
        } else {
            return OptimizationResult3D::Infeasible {
                last_optimal_velocity: optimal_velocity,
            };
        }
    }

    OptimizationResult3D::Feasible { optimal_velocity }
}
