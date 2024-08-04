use std::f32::consts;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use steering::{follow_path, update_agent_on_path};

#[derive(Component)]
struct Velocity {
    pub value: Vec3,
}

#[derive(Component)]
struct FollowPath {
    pub path: Vec<Vec3>,
}

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

fn setup(
    mut commands: Commands,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let ship_material = StandardMaterial {
        base_color_texture: Some(asset_server.load("textures/shuttle_BaseColor.png")),
        metallic_roughness_texture: Some(
            asset_server.load("textures/shuttle_MetallicRoughness.png"),
        ),
        normal_map_texture: Some(asset_server.load("textures/shuttle_Normal.png")),
        occlusion_texture: Some(asset_server.load("textures/shuttle_Occlusion.png")),
        ..Default::default()
    };

    let ship_material = standard_materials.add(ship_material);

    let ship = commands
        .spawn(PbrBundle {
            mesh: asset_server.load("models/shuttle.glb#Mesh0/Primitive0"),
            material: ship_material,
            transform: Transform::from_translation(Vec3::ZERO)
                .with_scale(Vec3::splat(10.0))
                .with_rotation(Quat::from_rotation_y(-consts::FRAC_PI_2)),
            ..Default::default()
        })
        .insert(Velocity { value: Vec3::ZERO })
        .insert(FollowPath {
            path: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1000.0, 0.0, 0.0),
                Vec3::new(800.0, 400.0, 0.0),
                Vec3::new(0.0, 400.0, 0.0),
                Vec3::new(300.0, 200.0, 100.0),
                Vec3::new(0.0, 800.0, 0.0),
                Vec3::new(300.0, 200.0, 100.0),
            ],
        })
        .id();

    let light = commands
        .spawn(DirectionalLightBundle {
            transform: Transform::from_translation(Vec3::ZERO),
            directional_light: DirectionalLight {
                illuminance: 3000.0,
                ..Default::default()
            },
            ..Default::default()
        })
        .id();

    commands
        .spawn((
            Camera3dBundle {
                ..Default::default()
            },
            UniversalCamera::Orbit {
                focus: CameraTarget::Entity(ship),
                offset: Vec3::ZERO,
                current_focus: Vec3::ZERO,
                radius: 1000.0,
                locked_cursor_position: None,
            },
        ))
        .add_child(light);
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Velocity, &mut FollowPath, &mut Transform)>,
    mut commands: Commands,
) {
    const TURNING_SPEED: f32 = 2.0;
    const MAX_SPEED: f32 = 100.0;
    const MAX_FORCE: f32 = 50.0;
    const AGENT_MASS: f32 = 1.0;

    for (entity, mut velocity, mut path, mut transform) in query.iter_mut() {
        let follow_path_result = follow_path(
            &path.path,
            0,
            transform.translation,
            velocity.value,
            TURNING_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            1.0,
            &mut gizmos,
        );

        let force = match follow_path_result {
            steering::FollowPathResult::CurrentSegment(force) => force,
            steering::FollowPathResult::NextSegment(force, segment) => {
                path.path = path.path.split_off(segment);
                force
            }
            steering::FollowPathResult::EndOfPath(force) => {
                commands.entity(entity).remove::<FollowPath>();
                force
            }
        };

        let (new_velocity, new_rotation) = update_agent_on_path(
            velocity.value,
            transform.rotation,
            TURNING_SPEED,
            MAX_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            force,
            time.delta_seconds(),
        );

        velocity.value = new_velocity;
        transform.rotation = new_rotation;

        transform.translation += velocity.value * time.delta_seconds();

        for i in 0..path.path.len() - 1 {
            gizmos.line(path.path[i], path.path[i + 1], Color::WHITE);
        }
    }
}
