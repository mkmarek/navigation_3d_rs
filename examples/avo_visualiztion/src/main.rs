use std::{f32::consts::E, ops::RangeInclusive};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{colliders::Collider, Sphere};
use orca::{AccelerationVelocityObstacle3D, Agent3D};
use ray_marching::{RayMarchData, RayMarchingPlugin};

mod ray_marching;

#[derive(Resource)]
struct AgentInformation {
    position_a: Vec3,
    position_b: Vec3,
    velocity_a: Vec3,
    velocity_b: Vec3,
    max_acceleration_a: f32,
    max_acceleration_b: f32,
    max_velocity_a: f32,
    max_velocity_b: f32,
    radius_a: f32,
    radius_b: f32,
    lookeahead: f32,
}

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
        .insert_resource(AgentInformation {
            position_a: Vec3::X * 100.0,
            position_b: Vec3::X * -100.0,
            velocity_a: Vec3::ZERO,
            velocity_b: Vec3::ZERO,
            max_acceleration_a: 100.0,
            max_acceleration_b: 100.0,
            max_velocity_a: 200.0,
            max_velocity_b: 200.0,
            radius_a: 50.0,
            radius_b: 50.0,
            lookeahead: 5.0,
        })
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                update_ray_march_data,
                draw_agent_gizmos,
                draw_ui,
                draw_curve,
                draw_velocity_obstacle,
            ),
        )
        .run();
}

fn setup(mut commands: Commands) {
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
}

fn draw_agent_gizmos(mut gizmos: Gizmos, agent_information: Res<AgentInformation>) {
    gizmos.sphere(
        agent_information.position_a,
        Quat::IDENTITY,
        agent_information.radius_a,
        Color::GREEN,
    );

    gizmos.line(
        agent_information.position_a,
        agent_information.position_a + agent_information.velocity_a,
        Color::GREEN,
    );

    gizmos.sphere(
        agent_information.position_b,
        Quat::IDENTITY,
        agent_information.radius_b,
        Color::RED,
    );

    gizmos.line(
        agent_information.position_b,
        agent_information.position_b + agent_information.velocity_b,
        Color::RED,
    );
}

fn update_ray_march_data(
    mut query: Query<(&GlobalTransform, &Camera, &mut RayMarchData)>,
    agent_information: Res<AgentInformation>,
) {
    for (transform, camera, mut ray_march_data) in query.iter_mut() {
        ray_march_data.projection = camera.projection_matrix();
        ray_march_data.projection_inverse = camera.projection_matrix().inverse();
        ray_march_data.view = transform.compute_matrix();
        ray_march_data.acceleration_ctrl_param =
            2.0 * agent_information.max_velocity_a / agent_information.max_acceleration_a;
        ray_march_data.e = E;
        ray_march_data.lookahead = agent_information.lookeahead;
        ray_march_data.velocity_ab = agent_information.velocity_a - agent_information.velocity_b;
        ray_march_data.position_ab = agent_information.position_a - agent_information.position_b;
        ray_march_data.velocity_b = agent_information.velocity_b;
        ray_march_data.radius_ab = agent_information.radius_a + agent_information.radius_b;
        ray_march_data.offset = agent_information.position_a;
    }
}

fn draw_ui(mut contexts: EguiContexts, mut info: ResMut<AgentInformation>) {
    const RANGE: RangeInclusive<f32> = -100.0..=100.0;

    egui::Window::new("Controls").show(contexts.ctx_mut(), |ui| {
        ui.add(egui::Slider::new(&mut info.position_a.x, RANGE).text("(A) X Position"));
        ui.add(egui::Slider::new(&mut info.position_a.y, RANGE).text("(A) Y Position"));
        ui.add(egui::Slider::new(&mut info.position_a.z, RANGE).text("(A) Z Position"));

        ui.add(egui::Slider::new(&mut info.velocity_a.x, RANGE).text("(A) X Velocity"));
        ui.add(egui::Slider::new(&mut info.velocity_a.y, RANGE).text("(A) Y Velocity"));
        ui.add(egui::Slider::new(&mut info.velocity_a.z, RANGE).text("(A) Z Velocity"));

        ui.add(egui::Slider::new(&mut info.position_b.x, RANGE).text("(B) X Position"));
        ui.add(egui::Slider::new(&mut info.position_b.y, RANGE).text("(B) Y Position"));
        ui.add(egui::Slider::new(&mut info.position_b.z, RANGE).text("(B) Z Position"));

        ui.add(egui::Slider::new(&mut info.velocity_b.x, RANGE).text("(B) X Velocity"));
        ui.add(egui::Slider::new(&mut info.velocity_b.y, RANGE).text("(B) Y Velocity"));
        ui.add(egui::Slider::new(&mut info.velocity_b.z, RANGE).text("(B) Z Velocity"));

        ui.add(
            egui::Slider::new(&mut info.max_acceleration_a, 0.0..=1000.0)
                .text("(A) Max Acceleration"),
        );
        ui.add(
            egui::Slider::new(&mut info.max_acceleration_b, 0.0..=1000.0)
                .text("(B) Max Acceleration"),
        );

        ui.add(egui::Slider::new(&mut info.max_velocity_a, 0.0..=1000.0).text("(A) Max Velocity"));
        ui.add(egui::Slider::new(&mut info.max_velocity_b, 0.0..=1000.0).text("(B) Max Velocity"));

        ui.add(egui::Slider::new(&mut info.radius_a, 0.0..=100.0).text("(A) Radius"));
        ui.add(egui::Slider::new(&mut info.radius_b, 0.0..=100.0).text("(B) Radius"));

        ui.add(egui::Slider::new(&mut info.lookeahead, 0.0..=100.0).text("Lookahead"));
    });
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn draw_curve(query: Query<&RayMarchData>, mut gizmos: Gizmos) {
    const AVO_SAMPLES: usize = 25;

    for info in query.iter() {
        let mut points = Vec::with_capacity(AVO_SAMPLES);
        for i in 0..AVO_SAMPLES {
            let t = lerp(1.0, info.lookahead, i as f32 / AVO_SAMPLES as f32);

            let param = info.acceleration_ctrl_param
                * (info.e.powf(-t / info.acceleration_ctrl_param) - 1.0);

            let position =
                info.offset + (param * info.velocity_ab - info.position_ab) / (t + param);

            points.push(position);
        }

        for i in 0..points.len() - 1 {
            gizmos.line(points[i], points[i + 1], Color::WHITE);
        }
    }
}

fn draw_velocity_obstacle(agent_information: Res<AgentInformation>, mut gizmos: Gizmos) {
    let mut agent_self = Agent3D::new(
        agent_information.position_a,
        agent_information.velocity_a,
        Collider::Sphere(Sphere::new(agent_information.radius_a, Vec3::ZERO)),
    );
    agent_self.responsibility = 1.0;

    let mut agent_other = Agent3D::new(
        agent_information.position_b,
        agent_information.velocity_b,
        Collider::Sphere(Sphere::new(agent_information.radius_b, Vec3::ZERO)),
    );
    agent_other.responsibility = 0.0;

    let avo = AccelerationVelocityObstacle3D::new(
        &agent_self,
        &agent_other,
        agent_information.lookeahead,
        2.0 * agent_information.max_velocity_a / agent_information.max_acceleration_a,
        25,
    );

    let orca = avo.orca_plane(0.1);

    gizmos.sphere(
        orca.origin + agent_information.position_a,
        Quat::IDENTITY,
        1.0,
        Color::BLUE,
    );

    gizmos.line(
        orca.origin + agent_information.position_a,
        orca.origin + agent_information.position_a + orca.normal * 100.0,
        Color::BLUE,
    );

    let transfrom =
        Transform::from_translation(agent_information.position_a).looking_to(orca.normal, Vec3::Y);

    gizmos.rect(
        orca.origin + agent_information.position_a,
        transfrom.rotation,
        Vec2::ONE * 100.0,
        Color::BLUE,
    );
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
