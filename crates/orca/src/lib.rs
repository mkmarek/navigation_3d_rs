#![warn(clippy::pedantic)]

pub(crate) const EPSILON: f32 = 0.0001;

mod agent_3d;
mod solver_2d;
mod solver_3d;
mod solver_4d;
mod velocity_obstacle_3d;

pub use agent_3d::*;
pub use velocity_obstacle_3d::*;

use bevy_math::{Vec3, Vec4};
use geometry::{Hyperplane, Plane, Sphere, Spherinder};
use solver_3d::{incremental_optimization_3d, OptimizationResult3D};
use solver_4d::{incremental_optimization_4d, OptimizationResult4D};

#[must_use]
pub fn optimize_velocity_3d(
    preffered_velocity: Vec3,
    maximum_velocity: f32,
    planes: &[Plane],
) -> Vec3 {
    let result = incremental_optimization_3d(
        preffered_velocity,
        &Sphere::new(maximum_velocity, Vec3::ZERO),
        planes,
    );

    match result {
        OptimizationResult3D::Feasible { optimal_velocity } => optimal_velocity,
        OptimizationResult3D::Infeasible {
            last_optimal_velocity: _,
        } => {
            let mut hyperplanes = Vec::with_capacity(planes.len());
            for plane in planes {
                let hyperplane = Hyperplane::new(
                    Vec4::new(plane.origin.x, plane.origin.y, plane.origin.z, 0.0),
                    Vec4::new(plane.normal.x, plane.normal.y, plane.normal.z, 0.2),
                );

                hyperplanes.push(hyperplane);
            }

            let result = incremental_optimization_4d(
                Vec4::new(
                    preffered_velocity.x,
                    preffered_velocity.y,
                    preffered_velocity.z,
                    -1000.0,
                ),
                &Spherinder::new(Vec4::ZERO, maximum_velocity),
                hyperplanes.as_slice(),
            );

            match result {
                OptimizationResult4D::Feasible { optimal_velocity } => optimal_velocity.truncate(),
                OptimizationResult4D::Infeasible {
                    last_optimal_velocity,
                } => last_optimal_velocity.truncate(),
            }
        }
    }
}
