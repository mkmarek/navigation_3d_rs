use bevy_math::Vec2;

use geometry::{
    HalfPlane, LineSegment2D, Ray2D, Ray2DIntersection, Ray2DIntersectionResult, Vec2Operations,
};

use crate::EPSILON;

#[derive(Debug)]
pub(crate) enum OptimizationResult2D {
    Feasible {
        optimal_velocity: Vec2,
    },
    #[allow(dead_code)]
    Infeasible {
        last_optimal_velocity: Vec2,
    },
}

pub trait MaximumVelocityShape2D {
    fn constrain(&self, velocity: Vec2) -> Vec2;
    fn get_bounds_on_line(&self, point: Vec2, direction: Vec2) -> Option<(f32, f32)>;
}

impl<T> MaximumVelocityShape2D for T
where
    T: Vec2Operations + Ray2DIntersection,
{
    fn constrain(&self, velocity: Vec2) -> Vec2 {
        self.constrain(velocity)
    }

    fn get_bounds_on_line(&self, point: Vec2, direction: Vec2) -> Option<(f32, f32)> {
        let ray = Ray2D::new(point, direction);
        match self.intersect(&ray) {
            Ray2DIntersectionResult::None => None,
            Ray2DIntersectionResult::Point(t) => Some((t, t)),
            Ray2DIntersectionResult::LineSegment(segment) => Some((segment.t_min, segment.t_max)),
        }
    }
}

pub(crate) fn incremental_optimization_2d(
    preffered_velocity: Vec2,
    maximum_velocity: &impl MaximumVelocityShape2D,
    half_planes: &[HalfPlane],
) -> OptimizationResult2D {
    let mut optimal_velocity = maximum_velocity.constrain(preffered_velocity);

    for i in 0..half_planes.len() {
        let half_plane = &half_planes[i];

        // Check if the velocity is inside the half plane
        // and if so, skip the optimization for this half plane.
        if half_plane.contains(optimal_velocity) {
            continue;
        }

        // The new optimal_velocity will lie on the half plane.
        // we will use the boundary direction of the half_plane and find a parameter t
        // initially the parameter t will be bound by the bounding circe defined
        // by the maximum velocity.

        let direction = half_plane.normal.perp();
        let point = half_plane.point;

        // If the itnersection won't find any points, we will skip this half plane.
        // This can happen if the half plane is completely outside the bounding circle.
        if let Some((mut min_bound, mut max_bound)) =
            maximum_velocity.get_bounds_on_line(point, direction)
        {
            // Now we will iterate over all previous half planes and constraint
            // the bounds of t to the intersection of the half planes.
            for half_plane_j in half_planes.iter().take(i) {
                let min_bound_point = point + direction * min_bound;
                let max_bound_point = point + direction * max_bound;

                // If the half plane contains the bounds of the bounding circle
                // we can skip this half plane.
                if half_plane_j.contains(min_bound_point) && half_plane_j.contains(max_bound_point)
                {
                    continue;
                }

                let ray_a = Ray2D::new(point, direction);
                let ray_b = Ray2D::new(half_plane_j.point, half_plane_j.normal.perp());

                if let Ray2DIntersectionResult::Point(line_intersection) = ray_a.intersect(&ray_b) {
                    if !half_plane_j.contains(min_bound_point) {
                        if line_intersection > min_bound {
                            min_bound = line_intersection;
                        } else {
                            return OptimizationResult2D::Infeasible {
                                last_optimal_velocity: optimal_velocity,
                            };
                        }
                    }

                    if !half_plane_j.contains(max_bound_point) {
                        if line_intersection < max_bound {
                            max_bound = line_intersection;
                        } else {
                            return OptimizationResult2D::Infeasible {
                                last_optimal_velocity: optimal_velocity,
                            };
                        }
                    }
                } else {
                    return OptimizationResult2D::Infeasible {
                        last_optimal_velocity: optimal_velocity,
                    };
                }
            }

            if (min_bound - max_bound).abs() < EPSILON {
                let avg = (min_bound + max_bound) / 2.0;
                min_bound = avg;
                max_bound = avg;
            }

            // If the bounds are invalid, we will return None as the optimization
            // is invalid.
            if min_bound > max_bound {
                return OptimizationResult2D::Infeasible {
                    last_optimal_velocity: optimal_velocity,
                };
            }

            // Now we have the bounds of t, we will find find the closest point on the half plane
            // within these bounds.
            let line_segment = LineSegment2D::new(point, direction, min_bound, max_bound);
            optimal_velocity = line_segment.constrain(optimal_velocity);
        } else if half_plane.contains(optimal_velocity) {
            // If the intersection is None, but the half plane contains the optimal velocity
            // we will skip this half plane.
            continue;
        } else {
            // If the intersection is None and the half plane doesn't contain the optimal velocity
            // we will return None as the optimization is invalid.
            return OptimizationResult2D::Infeasible {
                last_optimal_velocity: optimal_velocity,
            };
        }
    }

    OptimizationResult2D::Feasible { optimal_velocity }
}
