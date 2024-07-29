use bevy_math::{Vec2, Vec3, Vec4};

// Defines operations of a shape that can be performed with a Vec2.
pub trait Vec2Operations {
    // Returns true if the point is inside the shape.
    fn contains(&self, pt: Vec2) -> bool;

    // Constrains a point to the shape.
    // If the point is outside the shape, it is projected onto the shape.
    // If the point is inside the shape, it is returned as is.
    fn constrain(&self, pt: Vec2) -> Vec2;

    // Returns the closest point on the shape and the normal at that point.
    // The normal is the direction that points away from the shape.
    // The closest point is the point on the surface of the shape that is closest to the input point.
    fn closest_point_and_normal(&self, pt: Vec2) -> (Vec2, Vec2);

    // Returns the signed distance from the point to the shape.
    // The sign of the distance indicates whether the point is inside or outside the shape.
    fn signed_distance(&self, pt: Vec2) -> f32;
}

// Defines operations of a shape that can be performed with a Vec3.
pub trait Vec3Operations {
    // Returns true if the point is inside the shape.
    fn contains(&self, pt: Vec3) -> bool;

    // Constrains a point to the shape.
    // If the point is outside the shape, it is projected onto the shape.
    // If the point is inside the shape, it is returned as is.
    fn constrain(&self, pt: Vec3) -> Vec3;

    // Returns the closest point on the shape and the normal at that point.
    // The normal is the direction that points away from the shape.
    // The closest point is the point on the surface of the shape that is closest to the input point.
    fn closest_point_and_normal(&self, pt: Vec3) -> (Vec3, Vec3);

    // Returns the signed distance from the point to the shape.
    // The sign of the distance indicates whether the point is inside or outside the shape.
    fn signed_distance(&self, pt: Vec3) -> f32;
}

// Defines operations of a shape that can be performed with a Vec4.
pub trait Vec4Operations {
    // Returns true if the point is inside the shape.
    fn contains(&self, pt: Vec4) -> bool;

    // Constrains a point to the shape.
    // If the point is outside the shape, it is projected onto the shape.
    // If the point is inside the shape, it is returned as is.
    fn constrain(&self, pt: Vec4) -> Vec4;

    // Returns the signed distance from the point to the shape.
    // The sign of the distance indicates whether the point is inside or outside the shape.
    fn signed_distance(&self, pt: Vec4) -> f32;
}
