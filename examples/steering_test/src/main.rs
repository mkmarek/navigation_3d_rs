use std::{f32::consts, ops::Range};

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{colliders::Collider, Plane, Sphere, Vec3Operations};
use orca::{optimize_velocity_3d, AccelerationVelocityObstacle3D, Agent3D};
use rand::{thread_rng, Rng};
use steering::{follow_path, separation, update_agent_on_path, FollowPathResult};

#[derive(Component)]
struct Velocity {
    pub value: Vec3,
}

#[derive(Component)]
struct FollowPath {
    pub path: Vec<Vec3>,
}

#[derive(Component)]
struct Obstacle {
    pub radius: f32,
    pub collider: Sphere,
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
        .add_systems(Update, (draw_gizmos, generate_path))
        .run();
}

const BOX_SIZE: f32 = 500.0;
const NUMBER_OF_OBSTACLES: usize = 8;
const TURNING_SPEED: f32 = 2.0;
const MAX_SPEED: f32 = 80.0;
const MAX_FORCE: f32 = 50.0;
const AGENT_MASS: f32 = 2.0;
const MAX_ACCELERATION: f32 = MAX_FORCE / AGENT_MASS;
const AGENT_RADIUS: f32 = 5.0;
const SEPARATION_RADIUS: f32 = AGENT_RADIUS * 1.0;
const ORCA_RADIUS: f32 = SEPARATION_RADIUS * 1.0;
const NUMBER_OF_NEIGHBORS: usize = 20;
const OBSTACLE_RADIUS: Range<f32> = 10.0..20.0;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
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
                .with_scale(Vec3::splat(AGENT_RADIUS / 2.0))
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

    let mut rng = thread_rng();

    for x in 0..NUMBER_OF_OBSTACLES {
        for y in 0..NUMBER_OF_OBSTACLES {
            for z in 0..NUMBER_OF_OBSTACLES {
                let x = x as f32 * BOX_SIZE / NUMBER_OF_OBSTACLES as f32;
                let y = y as f32 * BOX_SIZE / NUMBER_OF_OBSTACLES as f32;
                let z = z as f32 * BOX_SIZE / NUMBER_OF_OBSTACLES as f32;

                let x = x + rng.gen_range(-50.0..50.0);
                let y = y + rng.gen_range(-50.0..50.0);
                let z = z + rng.gen_range(-50.0..50.0);
                let radius = rng.gen_range(OBSTACLE_RADIUS);

                commands
                    .spawn(PbrBundle {
                        mesh: meshes.add(
                            shape::UVSphere {
                                radius,
                                ..Default::default()
                            }
                            .into(),
                        ),
                        material: standard_materials.add(StandardMaterial {
                            base_color: Color::rgb(0.5, 0.5, 0.5),
                            ..Default::default()
                        }),
                        transform: Transform::from_translation(Vec3::new(x, y, z)),
                        ..Default::default()
                    })
                    .insert(Obstacle {
                        radius,
                        collider: Sphere::new(radius, Vec3::new(x, y, z)),
                    });
            }
        }
    }
}

fn generate_path(
    mut commands: Commands,
    query: Query<(Entity, &Transform, &Agent), Without<FollowPath>>,
) {
    let mut rng = thread_rng();

    for (entity, transform, _) in query.iter() {
        let path_length = rng.gen_range(5..10);

        let mut path = vec![];
        path.push(transform.translation);

        for _ in 0..path_length {
            path.push(Vec3::new(
                rng.gen_range(0.0..BOX_SIZE),
                rng.gen_range(0.0..BOX_SIZE),
                rng.gen_range(0.0..BOX_SIZE),
            ));
        }

        commands.entity(entity).insert(FollowPath { path });
    }
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Velocity, &mut FollowPath, &mut Transform), Without<Obstacle>>,
    obstacles: Query<(&Transform, &Obstacle)>,
    mut commands: Commands,
) {
    for (entity, mut velocity, mut path, mut transform) in query.iter_mut() {
        for (t, obstacle) in obstacles.iter() {
            let distance = t.translation.distance(transform.translation);

            if distance < AGENT_RADIUS + obstacle.radius {
                println!(
                    "Collision detected! {} < {}",
                    distance,
                    AGENT_RADIUS + obstacle.radius
                );
            }
        }

        let follow_path_result = follow_path(
            &path.path,
            0,
            transform.translation,
            velocity.value,
            TURNING_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            10.0,
        );

        println!("{:?}", follow_path_result);

        let mut desired_velocity = match follow_path_result {
            FollowPathResult::CurrentSegment(velocity) => velocity.clamp_length_max(MAX_SPEED),
            FollowPathResult::NextSegment(velocity, segment) => {
                path.path = path.path.split_off(segment);

                if path.path.is_empty() {
                    commands.entity(entity).remove::<FollowPath>();
                }

                velocity.clamp_length_max(MAX_SPEED)
            }
            FollowPathResult::EndOfPath(velocity) => {
                commands.entity(entity).remove::<FollowPath>();
                velocity.clamp_length_max(MAX_SPEED)
            }
        };

        let time_horizon = MAX_SPEED / (MAX_FORCE / AGENT_MASS);
        let mut nearest_neighbors = obstacles.iter().collect::<Vec<_>>();

        nearest_neighbors.sort_by(|(_, a), (_, b)| {
            let distance_a = a.collider.signed_distance(transform.translation);
            let distance_b = b.collider.signed_distance(transform.translation);

            distance_a.partial_cmp(&distance_b).unwrap()
        });

        let orca_planes = nearest_neighbors
            .iter()
            .take(NUMBER_OF_NEIGHBORS)
            .map(|a| {
                let mut self_agent = Agent3D::new(
                    transform.translation,
                    velocity.value,
                    Collider::Sphere(Sphere::new(ORCA_RADIUS, Vec3::ZERO)),
                );

                self_agent.responsibility = 1.0;

                let mut other_agent = Agent3D::new(
                    a.0.translation,
                    Vec3::ZERO,
                    Collider::Sphere(Sphere::new(a.1.collider.radius, Vec3::ZERO)),
                );

                other_agent.responsibility = 0.0;

                gizmos.sphere(
                    a.1.collider.origin,
                    Quat::IDENTITY,
                    a.1.collider.radius,
                    Color::WHITE,
                );

                AccelerationVelocityObstacle3D::new(
                    &self_agent,
                    &other_agent,
                    time_horizon,
                    2.0 * MAX_SPEED / MAX_ACCELERATION,
                    25,
                )
                .orca_plane(time.delta_seconds())
            })
            .collect::<Vec<Plane>>();

        let optimal_velocity = optimize_velocity_3d(
            desired_velocity - velocity.value,
            MAX_ACCELERATION * 2.0 * MAX_SPEED / MAX_ACCELERATION,
            orca_planes.as_slice(),
        );

        println!("Optimal velocity: {:?}", optimal_velocity);
        println!("Desired velocity: {:?}", desired_velocity);

        desired_velocity = velocity.value + optimal_velocity;

        println!("[updated] Desired velocity: {:?}", desired_velocity);

        let (new_velocity, new_rotation) = update_agent_on_path(
            velocity.value,
            transform.rotation,
            TURNING_SPEED,
            MAX_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            desired_velocity,
            time.delta_seconds(),
        );

        let obstacle_positions = obstacles
            .iter()
            .map(|(transform, obstacle)| (transform.translation, obstacle.collider.radius))
            .collect::<Vec<_>>();

        let separation_velocity = separation(
            transform.translation,
            &obstacle_positions,
            SEPARATION_RADIUS,
        );

        velocity.value = new_velocity;
        transform.rotation = new_rotation;
        transform.translation += (velocity.value + separation_velocity) * time.delta_seconds();

        for i in 0..path.path.len() - 1 {
            gizmos.line(path.path[i], path.path[i + 1], Color::WHITE);
        }

        gizmos.sphere(
            transform.translation,
            Quat::IDENTITY,
            AGENT_RADIUS,
            Color::RED,
        );

        //gizmos.sphere(
        //    transform.translation,
        //    Quat::IDENTITY,
        //    SEPARATION_RADIUS,
        //    Color::GREEN,
        //);

        //gizmos.sphere(
        //    transform.translation,
        //    Quat::IDENTITY,
        //    ORCA_RADIUS,
        //    Color::BLUE,
        //);
    }
}
