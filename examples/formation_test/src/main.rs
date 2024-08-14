use std::f32::consts;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};

#[derive(Component)]
struct Velocity {
    #[allow(dead_code)]
    pub value: Vec3,
}

#[derive(Component)]
struct Agent;

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
        .run();
}

fn setup(
    mut commands: Commands,
    mut _meshes: ResMut<Assets<Mesh>>,
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

    let initial_position = Vec3::new(-100.0, 0.0, 0.0);
    let ship = commands
        .spawn(PbrBundle {
            mesh: asset_server.load("models/shuttle.glb#Mesh0/Primitive0"),
            material: ship_material,
            transform: Transform::from_translation(initial_position)
                .with_scale(Vec3::splat(1.0))
                .with_rotation(Quat::from_rotation_y(-consts::FRAC_PI_2)),
            ..Default::default()
        })
        .insert(Velocity { value: Vec3::ZERO })
        .insert(Agent)
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
