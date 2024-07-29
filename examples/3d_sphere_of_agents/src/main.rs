use std::f32::consts::PI;

use bevy::{prelude::*, render::mesh::shape::UVSphere};
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{colliders::Collider, Plane};
use orca::{optimize_velocity_3d, Agent3D, VelocityObstacle3D};

#[derive(Debug, Clone, Copy, Resource, Default)]
struct Statistics {
    number_of_collisions: usize,
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
        .init_resource::<Statistics>()
        .add_systems(Startup, setup)
        .add_systems(Update, update_agents)
        .run();
}

#[derive(Component)]
struct Agent {
    shape: Collider,
    target_position: Vec3,
    velocity: Vec3,
    last_updated: Option<f32>,
}

fn spawn_agent(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    radius: f32,
    position: Vec3,
    target_position: Vec3,
) {
    commands
        .spawn((PbrBundle {
            mesh: meshes.add(UVSphere::default().into()),
            material: materials.add(Color::GREEN.into()),
            transform: Transform::from_translation(position).with_scale(Vec3::splat(radius)),
            ..default()
        },))
        .insert(Agent {
            shape: Collider::new_sphere(radius),
            velocity: Vec3::ZERO,
            target_position,
            last_updated: None,
        });
}

const AGENT_SPEED: f32 = 100.0;
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    const N_AGENTS: usize = 2000;
    let positions = sample_points_on_sphere(N_AGENTS, 800.0);

    for pos in positions {
        spawn_agent(&mut commands, &mut meshes, &mut materials, 10.0, pos, -pos);
    }

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

    commands.spawn(DirectionalLightBundle { ..default() });
}

fn arrive_velocity(current_position: Vec3, target_position: Vec3, max_speed: f32) -> Vec3 {
    let desired_velocity = (target_position - current_position).normalize() * max_speed;
    let distance = (target_position - current_position).length();
    if distance < 1.0 {
        return Vec3::ZERO;
    }
    if distance < max_speed {
        return desired_velocity * (distance / max_speed);
    }
    desired_velocity
}

fn round_to_precision(value: Vec3, precision: f32) -> Vec3 {
    (value / precision).round() * precision
}

fn separation_velocity(
    current_position: Vec3,
    agents: &[&Agent3D],
    separation_distance: f32,
) -> Vec3 {
    let mut separation_velocity = Vec3::ZERO;
    for agent in agents {
        let distance = (current_position - agent.position).length();
        if distance < separation_distance {
            separation_velocity +=
                (current_position - agent.position).normalize() * (separation_distance - distance);
        }
    }
    separation_velocity
}

fn update_agents(
    time: Res<Time>,
    mut agents: Query<(Entity, &mut Agent, &mut Transform)>,
    mut statistics: ResMut<Statistics>,
) {
    const PRECISION: f32 = 0.01;
    const TIME_HORIZON: f32 = 12.0;
    const TIME_STEP: f32 = 0.1;

    let agent_instances = agents
        .iter()
        .map(|a| {
            (
                a.0,
                Agent3D::new(
                    round_to_precision(a.2.translation, PRECISION),
                    round_to_precision(a.1.velocity, PRECISION),
                    a.1.shape.clone(),
                ),
            )
        })
        .collect::<Vec<_>>();

    let mut number_of_collisions = 0;
    let mut minimum_distance = 100.0;
    for (entity, mut agent, mut transform) in agents.iter_mut() {
        let self_agent = Agent3D::new(
            round_to_precision(transform.translation, PRECISION),
            round_to_precision(agent.velocity, PRECISION),
            agent.shape.clone(),
        );
        let last_updated = agent.last_updated.unwrap_or(0.0);
        let other_agents = agent_instances
            .iter()
            .filter(|(e, _)| *e != entity)
            .map(|(_, a)| a)
            .collect::<Vec<&Agent3D>>();

        if time.elapsed_seconds() - last_updated > TIME_STEP {
            agent.last_updated = Some(time.elapsed_seconds());

            const NUMBER_OF_NEIGHBORS: usize = 15;

            // Get number of nearest neighbors
            let mut nearest_neighbors = other_agents
                .iter()
                .filter(|a| {
                    let distance = (self_agent.position - a.position).length();

                    if distance < 20.0 {
                        number_of_collisions += 1;

                        if distance < minimum_distance {
                            minimum_distance = distance;
                        }
                    }

                    distance < AGENT_SPEED * TIME_HORIZON * TIME_STEP
                })
                .collect::<Vec<_>>();

            nearest_neighbors.sort_by(|a, b| {
                let distance_a = (self_agent.position - a.position).length();
                let distance_b = (self_agent.position - b.position).length();
                distance_a.partial_cmp(&distance_b).unwrap()
            });

            let desired_velocity =
                arrive_velocity(transform.translation, agent.target_position, AGENT_SPEED);

            let orca_planes = nearest_neighbors
                .iter()
                .take(NUMBER_OF_NEIGHBORS)
                //.map(|a| create_orca_plane(&self_agent, a, TIME_HORIZON, TIME_STEP))
                //    .collect::<Vec<Plane>>();
                .map(|a| {
                    VelocityObstacle3D::new(&self_agent, a, TIME_HORIZON)
                        .orca_plane(time.delta_seconds().max(TIME_STEP))
                })
                .collect::<Vec<Plane>>();

            let mut optimal_velocity =
                optimize_velocity_3d(desired_velocity, AGENT_SPEED, orca_planes.as_slice());

            if optimal_velocity.length() < desired_velocity.length() * 0.2 {
                let desired_velocity = desired_velocity.cross(Vec3::Y);

                optimal_velocity =
                    optimize_velocity_3d(desired_velocity, AGENT_SPEED, orca_planes.as_slice());
            }

            agent.velocity = agent.velocity.lerp(optimal_velocity, 0.3);
        }

        if number_of_collisions > statistics.number_of_collisions {
            statistics.number_of_collisions = number_of_collisions;
        }

        let separation_velocity =
            separation_velocity(transform.translation, other_agents.as_slice(), 50.0);
        agent.velocity += separation_velocity;

        if agent.velocity.length() > AGENT_SPEED {
            agent.velocity = agent.velocity.normalize() * AGENT_SPEED;
        }

        transform.translation += agent.velocity * time.delta_seconds();
    }

    //println!("Number of collisions: {}", statistics.number_of_collisions);
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
