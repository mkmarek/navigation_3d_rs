use std::ops::Range;

use bevy::{core_pipeline::clear_color::ClearColorConfig, prelude::*};
use bevy_egui::EguiPlugin;
use coordination::{
    best_matching_indexes,
    formations::{CircleFormation, LineFormation, QueueFormation, VFormation},
    Formation, FormationTemplate, FormationTemplateSet,
};
use example_utils::{
    CameraTarget, SkyboxPlugin, UniversalCamera, UniversalCameraPlugin, UtilsPlugin,
};
use geometry::{colliders::Collider, Sphere};
use orca::{optimize_velocity_3d, AccelerationVelocityObstacle3D, Agent3D};
use rand::{thread_rng, Rng};
use steering::{arrive, follow_path, update_agent_on_path, FollowPathResult};

#[derive(Component)]
struct Velocity {
    #[allow(dead_code)]
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
struct Agent {
    pub destination: Vec3,
}

#[derive(Component)]
struct FormationComponent {
    pub formation: Formation,
    pub agents: Vec<Entity>,
    pub formation_templates: Vec<Box<dyn FormationTemplate + Send + Sync>>,
}

const BOX_SIZE: f32 = 500.0;
const NUMBER_OF_OBSTACLES: usize = 8;
const TURNING_SPEED: f32 = 2.0;
const MAX_SPEED: f32 = 80.0;
const MAX_FORCE: f32 = 50.0;
const AGENT_MASS: f32 = 2.0;
const MAX_ACCELERATION: f32 = MAX_FORCE / AGENT_MASS;
const AGENT_RADIUS: f32 = 5.0;
const NUMBER_OF_AGENTS: usize = 10;
const SEPARATION_RADIUS: f32 = AGENT_RADIUS * 1.0;
const ORCA_RADIUS: f32 = SEPARATION_RADIUS * 1.0;
const NUMBER_OF_NEIGHBORS: usize = 20;
const OBSTACLE_RADIUS: Range<f32> = 10.0..20.0;

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
            SkyboxPlugin,
            EguiPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                generate_path,
                move_formation_along_path,
                move_agents_to_position,
                update_formation_info,
                print_path,
            ),
        )
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

    let initial_position = Vec3::new(
        -BOX_SIZE / 2.0 - ORCA_RADIUS * NUMBER_OF_AGENTS as f32,
        0.0,
        -BOX_SIZE - AGENT_RADIUS * 2.0,
    );

    let mut ships = Vec::new();
    let mut positions = Vec::new();
    for i in 0..NUMBER_OF_AGENTS {
        let position = initial_position + Vec3::new(0.0, 0.0, i as f32 * AGENT_RADIUS * 2.0);
        positions.push(position);
        ships.push(
            commands
                .spawn(PbrBundle {
                    mesh: asset_server.load("models/shuttle.glb#Mesh0/Primitive0"),
                    material: ship_material.clone(),
                    transform: Transform::from_translation(position)
                        .with_scale(Vec3::splat(AGENT_RADIUS / 2.0)),
                    ..Default::default()
                })
                .insert(Velocity { value: Vec3::ZERO })
                .id(),
        );
    }

    commands.spawn((
        FormationComponent {
            formation: Formation::new(positions),
            agents: ships.clone(),
            formation_templates: vec![
                Box::new(CircleFormation::new(ORCA_RADIUS, 15.0, 3.0)),
                Box::new(LineFormation::new(ORCA_RADIUS, 15.0, 12.0)),
                Box::new(VFormation::new(ORCA_RADIUS, 15.0, 9.0)),
                Box::new(QueueFormation::new(ORCA_RADIUS, 15.0, 1.0)),
            ],
        },
        Velocity { value: Vec3::ZERO },
    ));

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
                camera_3d: Camera3d {
                    clear_color: ClearColorConfig::None,
                    ..Default::default()
                },
                ..Default::default()
            },
            UniversalCamera::Orbit {
                focus: CameraTarget::Entity(ships[0]),
                offset: Vec3::ZERO,
                current_focus: Vec3::ZERO,
                radius: 1000.0,
                locked_cursor_position: None,
            },
        ))
        .add_child(light);
}

fn generate_path(
    mut commands: Commands,
    formations: Query<(Entity, &FormationComponent), Without<FollowPath>>,
) {
    let mut rng = thread_rng();

    for (entity, formation) in formations.iter() {
        let path_length = rng.gen_range(5..10);

        let mut path = vec![];
        path.push(formation.formation.get_bounds(ORCA_RADIUS).center);

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

fn move_formation_along_path(
    mut gizmos: Gizmos,
    time: Res<Time>,
    mut formations: Query<(Entity, &mut FollowPath, &FormationComponent, &mut Velocity)>,
    obstacles: Query<(&Transform, &Obstacle)>,
    mut commands: Commands,
) {
    for (entity, mut path, formation, mut velocity) in formations.iter_mut() {
        let formation_center = formation.formation.get_bounds(ORCA_RADIUS).center;
        let follow_path_result = follow_path(
            &path.path,
            0,
            formation_center,
            velocity.value,
            TURNING_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            10.0,
        );

        let desired_velocity = match follow_path_result {
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

        let template_set =
            FormationTemplateSet::from_iter(formation.formation_templates.iter().map(|e| {
                let template: &dyn FormationTemplate = e.as_ref();

                template
            }));

        let (best_formation, best_velocity) = template_set.get_best_formation_and_velocity(
            formation.formation.get_positions(),
            desired_velocity,
            MAX_SPEED,
            0.02f32,
            &[],
            5.0,
            25,
            25,
            200,
        );

        velocity.value = best_velocity;
        println!("Best formation: {:?}", best_formation);

        let formation_rotation = Quat::from_rotation_arc(Vec3::Z, best_velocity.normalize());
        let new_positions = best_formation
            .get_positions()
            .iter()
            .map(|position| {
                (if formation_rotation.is_near_identity() {
                    *position
                } else {
                    formation_rotation * *position
                }) + best_velocity
                    + formation_center
            })
            .collect::<Vec<_>>();

        let best_matches =
            best_matching_indexes(formation.formation.get_positions(), &new_positions);

        for (agent_index, new_position) in best_matches {
            let agent = formation.agents[agent_index];
            let agent_position = new_positions[new_position];

            commands.entity(agent).insert(Agent {
                destination: agent_position,
            });

            gizmos.line(
                formation.formation.get_positions()[agent_index],
                agent_position,
                Color::WHITE,
            );

            gizmos.sphere(agent_position, Quat::IDENTITY, 1.0, Color::WHITE);
        }
    }
}

fn move_agents_to_position(
    time: Res<Time>,
    mut gizmos: Gizmos,
    mut agents: Query<(Entity, &mut Transform, &Agent, &mut Velocity)>,
    obstacles: Query<(&Transform, &Obstacle), Without<Agent>>,
) {
    let agent_instances = agents
        .iter()
        .map(|(entity, transform, _, velocity)| {
            (
                entity,
                Agent3D::new(
                    transform.translation,
                    velocity.value,
                    Collider::new_sphere(ORCA_RADIUS),
                ),
            )
        })
        .collect::<Vec<_>>();

    let time_horizon = MAX_SPEED / (MAX_FORCE / AGENT_MASS);
    for (entity, mut agent_transform, agent, mut velocity) in agents.iter_mut() {
        let desired_velocity = arrive(
            agent.destination,
            agent_transform.translation,
            AGENT_MASS,
            MAX_FORCE,
            1.0,
        );

        let self_agent = Agent3D::new(
            agent_transform.translation,
            velocity.value,
            Collider::new_sphere(ORCA_RADIUS),
        );

        let orca_planes = agent_instances
            .iter()
            .filter_map(|(other_entity, other_agent)| {
                if entity == *other_entity {
                    return None;
                }

                Some(
                    AccelerationVelocityObstacle3D::new(
                        &self_agent,
                        other_agent,
                        time_horizon,
                        2.0 * MAX_SPEED / MAX_ACCELERATION,
                        25,
                    )
                    .orca_plane(time.delta_seconds()),
                )
            })
            .collect::<Vec<_>>();

        let optimal_velocity = optimize_velocity_3d(
            desired_velocity - velocity.value,
            MAX_ACCELERATION * 2.0 * MAX_SPEED / MAX_ACCELERATION,
            orca_planes.as_slice(),
        );

        let desired_velocity = velocity.value + optimal_velocity;

        let (new_velocity, new_rotation) = update_agent_on_path(
            velocity.value,
            agent_transform.rotation,
            TURNING_SPEED,
            MAX_SPEED,
            MAX_FORCE,
            AGENT_MASS,
            desired_velocity,
            time.delta_seconds(),
        );

        velocity.value = new_velocity;
        agent_transform.rotation = new_rotation;
        agent_transform.translation += velocity.value * time.delta_seconds();
    }
}

fn update_formation_info(
    mut formations: Query<&mut FormationComponent>,
    agents: Query<&Transform>,
) {
    for mut formation in formations.iter_mut() {
        let mut positions = Vec::new();

        for agent in formation.agents.iter() {
            positions.push(agents.get(*agent).unwrap().translation);
        }

        formation.formation = Formation::new(positions);
    }
}

fn print_path(mut gizmos: Gizmos, formations: Query<&FollowPath>) {
    for path in formations.iter() {
        for i in 0..path.path.len() - 1 {
            gizmos.line(path.path[i], path.path[i + 1], Color::WHITE);
        }
    }
}
