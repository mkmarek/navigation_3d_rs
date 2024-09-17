use bevy_math::Vec4;

use geometry::{Hyperplane, HyperplaneIntersection, Plane, Vec4Operations};

use crate::{incremental_optimization_3d, solver_3d::MaximumVelocityShape3D, OptimizationResult3D};

#[derive(Debug)]
pub enum OptimizationResult4D {
    Feasible { optimal_velocity: Vec4 },
    Infeasible { last_optimal_velocity: Vec4 },
}

pub trait MaximumVelocityShape4D {
    fn constrain(&self, velocity: Vec4) -> Vec4;
    fn project_on_hyperplane(&self, plane: &Hyperplane) -> Option<impl MaximumVelocityShape3D>;
}

impl<T> MaximumVelocityShape4D for T
where
    T: Vec4Operations + HyperplaneIntersection,
{
    fn constrain(&self, velocity: Vec4) -> Vec4 {
        self.constrain(velocity)
    }

    fn project_on_hyperplane(&self, plane: &Hyperplane) -> Option<impl MaximumVelocityShape3D> {
        self.intersect(plane)
    }
}

#[allow(clippy::missing_panics_doc)]
pub fn incremental_optimization_4d(
    preffered_velocity: Vec4,
    bounding_shape: &impl MaximumVelocityShape4D,
    hyperplanes: &[Hyperplane],
) -> OptimizationResult4D {
    let mut optimal_velocity = bounding_shape.constrain(preffered_velocity);

    for i in 0..hyperplanes.len() {
        let hyperplane = &hyperplanes[i];

        // Check if the velocity is inside the the zone of the plane
        // If it is, we can skip this plane
        if hyperplane.contains(optimal_velocity) {
            continue;
        }

        // The new optimal_velocity will lie somewhere on the hyperplane
        // first we find a intersecting 3d shape between the hyperplane and the bounding shape
        // then we calculate intersections of all the previous hyperplanes with the current one
        // which will yield an array of planes. We use all of that to solve a 3d optimization
        // problem, which will give us the optimal velocity on the hyperplane
        let mut planes = Vec::new();
        let bounding_shape_3d = bounding_shape.project_on_hyperplane(hyperplane);

        if bounding_shape_3d.is_none() {
            return OptimizationResult4D::Infeasible {
                last_optimal_velocity: optimal_velocity,
            };
        }

        let bounding_shape_3d = bounding_shape_3d.unwrap();

        let optimal_velocity_on_hyperplane = hyperplane.constrain(optimal_velocity);
        let optimal_velocity_on_hyperplane = hyperplane.project_3d(optimal_velocity_on_hyperplane);

        for hyperplaneplane_j in hyperplanes.iter().take(i) {
            if let Some(plane) = Plane::from_hyperplane_intersection(hyperplane, hyperplaneplane_j)
            {
                planes.push(plane);
            }
        }

        let result = incremental_optimization_3d(
            optimal_velocity_on_hyperplane,
            &bounding_shape_3d,
            &planes,
        );

        if let OptimizationResult3D::Feasible {
            optimal_velocity: opt,
        } = result
        {
            optimal_velocity = hyperplane.project_4d(opt);
        } else {
            return OptimizationResult4D::Infeasible {
                last_optimal_velocity: optimal_velocity,
            };
        }
    }

    OptimizationResult4D::Feasible { optimal_velocity }
}
