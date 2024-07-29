use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{InfiniteCone, Vec3Operations};

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
        .add_systems(Update, draw_gizmos)
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

fn draw_gizmos(mut gizmos: Gizmos) {
    const N_POINTS: usize = 100;

    let vertex = Vec3::new(10.0, 10.0, 5.0);
    let direction = Vec3::X * 1000.0;
    let radius = 500.0;

    let cone = InfiniteCone::new(vertex, direction, radius);

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
