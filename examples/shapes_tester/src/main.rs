use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{Cone, Vec3Operations};

fn main() {
    App::new()
        .insert_resource(Msaa::default())
        .add_plugins((
            DefaultPlugins.set(AssetPlugin {
                file_path: "../../assets".to_string(),
                ..Default::default()
            }),
            UtilsPlugin,
            UniversalCameraPlugin,
            EguiPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, draw_truncated_cone_gizmos)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3dBundle {
            ..Default::default()
        },
        UniversalCamera::Orbit {
            focus: CameraTarget::Position(Vec3::ZERO),
            offset: Vec3::ZERO,
            current_focus: Vec3::ZERO,
            radius: 1000.0,
            locked_cursor_position: None,
        },
    ));
}

#[allow(dead_code)]
fn draw_cone_gizmos(mut gizmos: Gizmos) {
    const N_POINTS: usize = 100;

    let vertex = Vec3::new(-200.0, 0.0, 0.0);
    let direction = Vec3::X * 1000.0;
    let radius = 0.5 * 1000.0;

    let cone = Cone::infinite(vertex, direction, radius);

    for point in sample_points_on_sphere(N_POINTS, 1000.0) {
        let (closest, normal) = cone.closest_point_and_normal(point);

        if cone.contains(point) {
            gizmos.sphere(point, Quat::IDENTITY, 5.0, Color::GREEN);
        } else {
            gizmos.sphere(point, Quat::IDENTITY, 5.0, Color::RED);
        }

        let signed_distance = cone.signed_distance(point);

        gizmos.sphere(closest, Quat::IDENTITY, 5.0, Color::BLUE);
        gizmos.line(closest, closest + normal * signed_distance, Color::BLUE);
    }

    draw_cone(&mut gizmos, vertex, direction, radius);
}

#[allow(dead_code)]
fn draw_truncated_cone_gizmos(mut gizmos: Gizmos) {
    const N_POINTS: usize = 100;

    let front_radius = 100.0;
    let back_radius = 150.0;
    let front_position = Vec3::new(50.0, 0.0, 0.0);
    let back_position = Vec3::new(150.0, 0.0, 0.0);

    let cone = Cone::new(front_radius, front_position, back_radius, back_position);

    for point in sample_points_on_sphere(N_POINTS, 1000.0) {
        let (closest, _) = cone.closest_point_and_normal(point);

        if cone.contains(point) {
            gizmos.sphere(point, Quat::IDENTITY, 5.0, Color::GREEN);
        } else {
            gizmos.sphere(point, Quat::IDENTITY, 5.0, Color::RED);
        }

        gizmos.sphere(closest, Quat::IDENTITY, 5.0, Color::BLUE);
        gizmos.line(closest, point, Color::BLUE);
    }

    draw_truncated_cone(
        &mut gizmos,
        front_radius,
        front_position,
        back_radius,
        back_position,
    );
}

#[allow(dead_code)]
fn draw_cone(gizmos: &mut Gizmos, vertex: Vec3, direction: Vec3, radius: f32) {
    const SEGMENTS: usize = 32;

    // Draw base
    gizmos.circle(
        vertex + direction,
        direction.normalize(),
        radius,
        Color::GREEN,
    );

    let dot_x = Vec3::X.dot(direction.normalize());
    let dot_y = Vec3::Y.dot(direction.normalize());
    let dot_z = Vec3::Z.dot(direction.normalize());

    let basis_angle = if dot_x.abs() < dot_y.abs() && dot_x.abs() < dot_z.abs() {
        Vec3::X
    } else if dot_y.abs() < dot_z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };

    // Draw sides
    for i in 0..SEGMENTS {
        let angle = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let perp_1 = direction.cross(basis_angle).normalize();
        let perp_2 = direction.cross(perp_1).normalize();

        let end =
            vertex + direction + (angle.cos() * radius * perp_1) + (angle.sin() * radius * perp_2);

        gizmos.line(vertex, end, Color::GREEN);
    }
}

#[allow(dead_code)]
fn draw_truncated_cone(
    gizmos: &mut Gizmos,
    front_radius: f32,
    front_position: Vec3,
    back_radius: f32,
    back_position: Vec3,
) {
    const SEGMENTS: usize = 32;

    let direction = (back_position - front_position).normalize();

    // Draw front base
    gizmos.circle(
        front_position,
        direction.normalize(),
        front_radius,
        Color::RED,
    );

    // Draw back base
    gizmos.circle(
        back_position,
        direction.normalize(),
        back_radius,
        Color::RED,
    );

    let dot_x = Vec3::X.dot(direction.normalize());
    let dot_y = Vec3::Y.dot(direction.normalize());
    let dot_z = Vec3::Z.dot(direction.normalize());

    let basis_angle = if dot_x.abs() < dot_y.abs() && dot_x.abs() < dot_z.abs() {
        Vec3::X
    } else if dot_y.abs() < dot_z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };

    // Draw sides
    for i in 0..SEGMENTS {
        let angle = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
        let perp_1 = direction.cross(basis_angle).normalize();
        let perp_2 = direction.cross(perp_1).normalize();

        let start = front_position
            + direction
            + (angle.cos() * front_radius * perp_1)
            + (angle.sin() * front_radius * perp_2);
        let end = back_position
            + direction
            + (angle.cos() * back_radius * perp_1)
            + (angle.sin() * back_radius * perp_2);

        gizmos.line(start, end, Color::RED);
    }
}

fn sample_points_on_sphere(n: usize, r: f32) -> Vec<Vec3> {
    let golden_ratio = (1.0 + 5.0_f32.sqrt()) / 2.0;
    let angle_increment = 2.0 * PI * golden_ratio;
    (0..n)
        .map(|i| {
            let y = 1.0 - (i as f32 / (n - 1) as f32) * 2.0; // y goes from 1 to -1
            let radius = (1.0 - y * y).sqrt() * r; // radius at y

            let theta = angle_increment * i as f32;
            let x = radius * theta.cos();
            let z = radius * theta.sin();
            Vec3::new(x, y * r, z)
        })
        .collect()
}
