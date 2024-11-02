use bevy::{prelude::*, render::mesh::shape::UVSphere};
use bevy_egui::EguiPlugin;
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{colliders::Collider, Plane};
use orca::{optimize_velocity_3d, AccelerationVelocityObstacle3D, Agent3D};

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
        .add_systems(Update, update_agents)
        .run();
}

#[derive(Component)]
struct Agent {
    shape: Collider,
    desired_velocity: Vec3,
    velocity: Vec3,
    last_updated: Option<f32>,
}

fn spawn_agent(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    radius: f32,
    position: Vec3,
    velocity: Vec3,
) {
    commands
        .spawn((PbrBundle {
            mesh: meshes.add(UVSphere::default().into()),
            material: materials.add(Color::GREEN.into()),
            transform: Transform::from_translation(position).with_scale(Vec3::splat(radius)),
            ..default()
        },))
        .insert(Agent {
            shape: Collider::new_sphere(radius * 2.0),
            velocity: Vec3::ZERO,
            desired_velocity: velocity,
            last_updated: None,
        });
}

const AGENT_SPEED: f32 = 100.0;
const MAX_FORCE: f32 = 100.0;
const AGENT_MASS: f32 = 2.0;
const MAX_ACCELERATION: f32 = MAX_FORCE / AGENT_MASS;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    const N_AGENTS: i32 = 2;
    const RADIUS: f32 = 15.0;

    for x in 0..N_AGENTS {
        for y in 0..N_AGENTS {
            for z in 0..N_AGENTS {
                spawn_agent(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    RADIUS,
                    Vec3::new(
                        -400.0 + (z as f32) * 30.0,
                        (x as f32) * 30.0,
                        (y as f32) * 30.0,
                    ),
                    Vec3::new(AGENT_SPEED, 0.0, 0.0),
                );

                spawn_agent(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    RADIUS,
                    Vec3::new(
                        400.0 - (z as f32) * 30.0,
                        (x as f32) * 30.0,
                        (y as f32) * 30.0,
                    ),
                    Vec3::new(-AGENT_SPEED, 0.0, 0.0),
                );
            }
        }
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
    //mut gizmos: Gizmos,
) {
    const TIME_HORIZON: f32 = 6.0;
    const TIME_STEP: f32 = 0.1;

    let agent_instances = agents
        .iter()
        .map(|a| {
            (
                a.0,
                Agent3D::new(a.2.translation, a.1.velocity, a.1.shape.clone()),
            )
        })
        .collect::<Vec<_>>();

    for (entity, mut agent, mut transform) in agents.iter_mut() {
        let self_agent = Agent3D::new(transform.translation, agent.velocity, agent.shape.clone());
        let last_updated = agent.last_updated.unwrap_or(0.0);
        let other_agents = agent_instances
            .iter()
            .filter(|(e, _)| *e != entity)
            .map(|(_, a)| a)
            .collect::<Vec<&Agent3D>>();

        if time.elapsed_seconds() - last_updated > TIME_STEP {
            agent.last_updated = Some(time.elapsed_seconds());

            const NUMBER_OF_NEIGHBORS: usize = 100;
            // Get number of nearest neighbors
            let mut nearest_neighbors = other_agents
                .iter()
                .filter(|a| {
                    let distance = (self_agent.position - a.position).length();

                    if distance < 20.0 {
                        println!("Distance: {}", distance);
                    }

                    true
                })
                .collect::<Vec<_>>();

            nearest_neighbors.sort_by(|a, b| {
                let distance_a = (self_agent.position - a.position).length();
                let distance_b = (self_agent.position - b.position).length();
                distance_a.partial_cmp(&distance_b).unwrap()
            });

            let orca_planes = nearest_neighbors
                .iter()
                .take(NUMBER_OF_NEIGHBORS)
                //.map(|a| create_orca_plane(&self_agent, a, TIME_HORIZON, TIME_STEP))
                //    .collect::<Vec<Plane>>();
                .filter_map(|a| {
                    AccelerationVelocityObstacle3D::new(
                        &self_agent,
                        a,
                        TIME_HORIZON,
                        2.0 * AGENT_SPEED / MAX_ACCELERATION,
                        25,
                    )
                    .orca_plane(time.delta_seconds().max(TIME_STEP))
                })
                .collect::<Vec<Plane>>();

            let optimal_velocity = optimize_velocity_3d(
                agent.desired_velocity - agent.velocity,
                MAX_ACCELERATION * 2.0 * AGENT_SPEED / MAX_ACCELERATION,
                orca_planes.as_slice(),
            );

            agent.velocity += optimal_velocity * MAX_ACCELERATION * time.delta_seconds();
        }

        let separation_velocity =
            separation_velocity(transform.translation, other_agents.as_slice(), 40.0);
        agent.velocity += separation_velocity;

        if agent.velocity.length() > AGENT_SPEED {
            agent.velocity = agent.velocity.normalize() * AGENT_SPEED;
        }

        transform.translation += agent.velocity * time.delta_seconds();
    }
}
