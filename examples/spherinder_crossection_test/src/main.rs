use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{
    CameraTarget, PlaneMaterial, UniversalCamera, UniversalCameraPlugin, UtilsPlugin,
};
use geometry::{Hyperplane, HyperplaneIntersection, Plane, Spherinder, Vec3Operations};
use ray_marching::{RayMarchData, RayMarchingPlugin};

mod ray_marching;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(AssetPlugin {
                file_path: "../../assets".to_string(),
                ..Default::default()
            }),
            UtilsPlugin,
            UniversalCameraPlugin,
            EguiPlugin,
            RayMarchingPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (update_ray_march_data, draw_ellipsoid, update_planes),
        )
        .run();
}

#[derive(Component)]
struct PlaneComponent {
    plane: Plane,
}

fn spawn_plane(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<PlaneMaterial>>,
    color: Color,
    origin: Vec3,
    normal: Vec3,
) {
    commands
        .spawn(MaterialMeshBundle {
            mesh: meshes.add(shape::Plane::from_size(1000.0).into()),
            material: materials.add(PlaneMaterial::new(color, 10.0)),
            ..default()
        })
        .insert(PlaneComponent {
            plane: Plane::new(origin, normal),
        });
}

fn setup(
    mut commands: Commands,
    mut assets: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<PlaneMaterial>>,
) {
    // camera
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
        RayMarchData { ..default() },
    ));

    spawn_plane(
        &mut commands,
        &mut assets,
        &mut materials,
        Color::WHITE,
        Vec3::new(3.4217021, -3.7875133, 5.4648843),
        Vec3::new(0.45757827, -0.50649756, 0.73080945),
    );

    spawn_plane(
        &mut commands,
        &mut assets,
        &mut materials,
        Color::GREEN,
        Vec3::new(-44.757824, 15.044995, -5.303109),
        Vec3::new(0.9419595, -0.3166324, 0.11160762),
    );
}

fn update_ray_march_data(
    time: Res<Time>,
    mut query: Query<(&GlobalTransform, &Camera, &mut RayMarchData)>,
) {
    for (transform, camera, mut ray_march_data) in query.iter_mut() {
        ray_march_data.projection = camera.projection_matrix();
        ray_march_data.projection_inverse = camera.projection_matrix().inverse();
        ray_march_data.view = transform.compute_matrix();
        ray_march_data.w = (time.elapsed_seconds() * 10.0) % 50.0;
    }
}

fn draw_ellipsoid(mut gizmos: Gizmos) {
    let hyperplane = Hyperplane::new(
        Vec4::new(-43.733166, 138.09503, -105.5708, 0.0),
        Vec4::new(-0.7777588, -0.12597124, -0.42334685, 0.44721353),
    );

    let spherinder = Spherinder::new(Vec4::ZERO, 100.0);
    let intersection = spherinder
        .intersect(&hyperplane)
        .expect("No intersection found");
    let points = sample_points_on_sphere(10, 1000.0);

    gizmos.sphere(
        Vec3::new(-61.435066, 35.541336, 66.35805),
        Quat::IDENTITY,
        5.0,
        Color::BLUE,
    );

    for point in points {
        gizmos.sphere(point, Quat::IDENTITY, 5.0, Color::RED);

        let constrained = intersection.constrain(point);

        gizmos.sphere(constrained, Quat::IDENTITY, 5.0, Color::GREEN);

        gizmos.line(point, constrained, Color::BLUE);
    }
}

fn update_planes(mut query: Query<(&PlaneComponent, &mut Transform)>, mut gizmos: Gizmos) {
    for (plane, mut transform) in query.iter_mut() {
        let plane = &plane.plane;
        *transform = Transform::from_translation(plane.origin).looking_to(plane.normal, Vec3::Y);
        transform.rotation *= Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);

        gizmos.line(plane.origin, plane.origin + plane.normal * 50.0, Color::RED);
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
