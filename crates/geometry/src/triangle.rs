use std::ops::Index;

use bevy_math::{Vec2, Vec3};

use crate::{LineSegment3D, Plane, Vec3Operations, EPSILON};

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

    pub fn centroid(&self) -> Vec3 {
        (self.points[0] + self.points[1] + self.points[2]) / 3.0
    }
}

impl Index<usize> for Triangle {
    type Output = Vec3;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}

impl Vec3Operations for Triangle {
    fn contains(&self, pt: Vec3) -> bool {
        // Move triangle so that p becomes origin
        let a = self.points[0] - pt;
        let b = self.points[1] - pt;
        let c = self.points[2] - pt;

        // Calculate normal vectors
        let u = b.cross(c);
        let v = c.cross(a);
        let w = a.cross(b);

        if u.dot(v) < -EPSILON {
            return false;
        }

        if u.dot(w) < -EPSILON {
            return false;
        }

        true
    }

    fn constrain(&self, pt: Vec3) -> Vec3 {
        let (closest_point, _) = self.closest_point_and_normal(pt);
        closest_point
    }

    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3) {
        let pt_projected = self.plane.constrain(pt);

        if self.contains(pt_projected) {
            (pt_projected, self.plane.normal)
        } else {
            let ab = LineSegment3D::from_two_points(self.points[0], self.points[1]);
            let bc = LineSegment3D::from_two_points(self.points[1], self.points[2]);
            let ca = LineSegment3D::from_two_points(self.points[2], self.points[0]);

            let closest_ab = ab.constrain(pt_projected);
            let closest_bc = bc.constrain(pt_projected);
            let closest_ca = ca.constrain(pt_projected);

            let distance_ab = (pt - closest_ab).length_squared();
            let distance_bc = (pt - closest_bc).length_squared();
            let distance_ca = (pt - closest_ca).length_squared();

            if distance_ab < distance_bc && distance_ab < distance_ca {
                let normal = (pt - closest_ab).normalize();
                if normal.dot(self.plane.normal) < 0.0 {
                    return (closest_ab, -normal);
                }
                (closest_ab, normal)
            } else if distance_bc < distance_ca {
                let normal = (pt - closest_bc).normalize();
                if normal.dot(self.plane.normal) < 0.0 {
                    return (closest_bc, -normal);
                }
                (closest_bc, normal)
            } else {
                let normal = (pt - closest_ca).normalize();
                if normal.dot(self.plane.normal) < 0.0 {
                    return (closest_ca, -normal);
                }
                (closest_ca, normal)
            }
        }
    }

    fn signed_distance(&self, pt: Vec3) -> f32 {
        let distance_to_plane = self.plane.signed_distance(pt).abs();
        let projected_pt = self.plane.constrain(pt);

        if self.contains(projected_pt) {
            // The point projects inside the triangle
            distance_to_plane
        } else {
            // The point projects outside the triangle
            let (closest_point, _) = self.closest_point_and_normal(pt);
            let distance_vector = pt - closest_point;
            distance_vector.length()
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_triangle_new_with_valid_points() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(1.0, 0.0, 0.0);
        let p3 = Vec3::new(0.0, 1.0, 0.0);
        let triangle = Triangle::new([p1, p2, p3]);

        assert_eq!(triangle.points, [p1, p2, p3]);
        assert_eq!(triangle.plane.normal, Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_triangle_new_with_duplicate_points() {
        let p = Vec3::new(0.0, 0.0, 0.0);
        let p3 = Vec3::new(1.0, 1.0, 1.0);
        let triangle = Triangle::new([p, p, p3]);

        // Verify handling of degenerate triangle
        // Assuming that Plane::from_points can handle this case
    }

    #[test]
    fn test_triangle_points_method() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(1.0, 0.0, 0.0);
        let p3 = Vec3::new(0.0, 1.0, 0.0);
        let triangle = Triangle::new([p1, p2, p3]);

        let points = triangle.points();
        assert_eq!(points, &[p1, p2, p3]);
    }

    #[test]
    fn test_triangle_normal_method() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]);

        let normal = triangle.normal();
        assert_relative_eq!(normal.x, 0.0);
        assert_relative_eq!(normal.y, 0.0);
        assert_relative_eq!(normal.z, 1.0);
    }

    #[test]
    fn test_triangle_centroid() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(0.0, 3.0, 0.0),
        ]);

        let centroid = triangle.centroid();
        assert_relative_eq!(centroid.x, 1.0);
        assert_relative_eq!(centroid.y, 1.0);
        assert_relative_eq!(centroid.z, 0.0);
    }

    #[test]
    fn test_triangle_indexing() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(1.0, 0.0, 0.0);
        let p3 = Vec3::new(0.0, 1.0, 0.0);
        let triangle = Triangle::new([p1, p2, p3]);

        assert_eq!(triangle[0], p1);
        assert_eq!(triangle[1], p2);
        assert_eq!(triangle[2], p3);
    }

    #[test]
    #[should_panic]
    fn test_triangle_index_out_of_bounds() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]);

        let _ = triangle[3];
    }

    #[test]
    fn test_triangle_contains_point_inside() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]);

        let pt_inside = Vec3::new(0.25, 0.25, 0.0);
        assert!(triangle.contains(pt_inside));
    }

    #[test]
    fn test_triangle_contains_point_on_edge() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(1.0, 0.0, 0.0);
        let p3 = Vec3::new(0.0, 1.0, 0.0);
        let triangle = Triangle::new([p1, p2, p3]);

        let pt_on_edge = (p1 + p2) * 0.5;
        assert!(triangle.contains(pt_on_edge));
    }

    #[test]
    fn test_triangle_contains_point_outside() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]);

        let pt_outside = Vec3::new(1.0, 1.0, 0.0);
        assert!(!triangle.contains(pt_outside));
    }

    #[test]
    fn test_triangle_constrain_point_inside() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(0.0, 3.0, 0.0),
        ]);

        let pt_inside = Vec3::new(1.0, 1.0, 0.0);
        let constrained_pt = triangle.constrain(pt_inside);
        assert_eq!(constrained_pt, pt_inside);
    }

    #[test]
    fn test_triangle_constrain_point_outside() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]);

        let pt_outside = Vec3::new(1.0, 1.0, 0.0);
        let constrained_pt = triangle.constrain(pt_outside);

        assert_relative_eq!(constrained_pt.x, 0.5);
        assert_relative_eq!(constrained_pt.y, 0.5);
        assert_relative_eq!(constrained_pt.z, 0.0);
    }

    #[test]
    fn test_triangle_closest_point_and_normal() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ]);

        let pt_outside = Vec3::new(1.0, 1.0, 1.0);
        let (closest_pt, normal) = triangle.closest_point_and_normal(pt_outside);

        assert_relative_eq!(closest_pt.x, 1.0);
        assert_relative_eq!(closest_pt.y, 1.0);
        assert_relative_eq!(closest_pt.z, 0.0);

        assert_relative_eq!(normal.x, 0.0);
        assert_relative_eq!(normal.y, 0.0);
        assert_relative_eq!(normal.z, 1.0);
    }

    #[test]
    fn test_triangle_signed_distance_above_plane() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ]);

        let pt_above = Vec3::new(1.0, 1.0, 1.0);
        let distance = triangle.signed_distance(pt_above);

        assert_relative_eq!(distance, 1.0);
    }

    #[test]
    fn test_triangle_signed_distance_below_plane() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ]);

        let pt_below = Vec3::new(1.0, 1.0, -1.0);
        let distance = triangle.signed_distance(pt_below);

        assert_relative_eq!(distance, 1.0);
    }

    #[test]
    fn test_triangle_signed_distance_inside_triangle() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(2.0, 0.0, 1.0),
            Vec3::new(0.0, 2.0, 1.0),
        ]);

        let pt_inside = Vec3::new(1.0, 1.0, 1.0);
        let distance = triangle.signed_distance(pt_inside);

        assert_relative_eq!(distance, 0.0);
    }

    #[test]
    fn test_triangle_signed_distance_on_plane_outside_triangle() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ]);

        let pt_outside = Vec3::new(3.0, 3.0, 0.0);
        let distance = triangle.signed_distance(pt_outside);
        let expected_distance = ((3.0_f32 - 1.0_f32).powi(2) + (3.0_f32 - 1.0_f32).powi(2)).sqrt();

        assert_relative_eq!(distance, expected_distance);
    }

    #[test]
    fn test_triangle_with_non_axis_aligned_plane() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(1.0, 0.0, 1.0);
        let p3 = Vec3::new(0.0, 1.0, 1.0);
        let triangle = Triangle::new([p1, p2, p3]);

        let normal = triangle.normal();
        let expected_normal = Vec3::new(-0.57735026, -0.57735026, 0.57735026);
        assert!((normal - expected_normal).length() < 1e-6);
    }

    #[test]
    fn test_triangle_with_degenerate_triangle_zero_area() {
        let p1 = Vec3::new(0.0, 0.0, 0.0);
        let triangle = Triangle::new([p1, p1, p1]);

        let pt = Vec3::new(1.0, 1.0, 1.0);
        let _distance = triangle.signed_distance(pt);
        // Check that methods do not panic
    }

    #[test]
    fn test_triangle_epsilons() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, EPSILON),
            Vec3::new(1.0, 0.0, EPSILON),
            Vec3::new(0.0, 1.0, EPSILON),
        ]);

        let pt = Vec3::new(0.25, 0.25, EPSILON / 2.0);
        assert!(triangle.contains(pt));
    }

    #[test]
    fn test_triangle_with_large_coordinates() {
        let large_value = 1e6_f32;
        let triangle = Triangle::new([
            Vec3::new(large_value, 0.0, 0.0),
            Vec3::new(0.0, large_value, 0.0),
            Vec3::new(0.0, 0.0, large_value),
        ]);

        let pt = Vec3::new(large_value / 3.0, large_value / 3.0, large_value / 3.0);
        assert!(triangle.contains(pt));
    }

    #[test]
    fn test_triangle_with_negative_coordinates() {
        let triangle = Triangle::new([
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(-2.0, -1.0, -1.0),
            Vec3::new(-1.0, -2.0, -1.0),
        ]);

        let pt_inside = Vec3::new(-1.5, -1.5, -1.0);
        assert!(triangle.contains(pt_inside));
    }

    #[test]
    fn test_triangle_integration_with_line_segment() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);

        let ab = LineSegment3D::from_two_points(triangle[0], triangle[1]);
        let pt_on_line = ab.constrain(Vec3::new(0.5, 0.5, 0.0));

        assert_relative_eq!(pt_on_line.x, 0.5);
        assert_relative_eq!(pt_on_line.y, 0.5);
        assert_relative_eq!(pt_on_line.z, 0.0);
    }

    #[test]
    fn test_triangle_consistency_across_methods() {
        let triangle = Triangle::new([
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ]);

        let pt = Vec3::new(0.5, 0.5, 0.0);
        assert!(triangle.contains(pt));

        let constrained_pt = triangle.constrain(pt);

        assert_relative_eq!(constrained_pt.x, 0.5);
        assert_relative_eq!(constrained_pt.y, 0.5);
        assert_relative_eq!(constrained_pt.z, 0.0);

        let distance = triangle.signed_distance(pt);
        assert_relative_eq!(distance, 0.0);
    }
}
