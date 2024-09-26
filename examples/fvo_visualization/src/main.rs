use std::ops::RangeInclusive;

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use example_utils::{CameraTarget, UniversalCamera, UniversalCameraPlugin, UtilsPlugin};
use geometry::{colliders::Collider, Sphere};
use orca::{Agent3D, FormationVelocityObstacle3D};

#[derive(Resource)]
struct AgentInformation {
    position_a: Vec3,
    position_b: Vec3,
    velocity_a: Vec3,
    velocity_b: Vec3,
    max_velocity_a: f32,
    max_velocity_b: f32,
    half_size_a: Vec3,
    radius_b: f32,
    lookeahead: f32,
    agent_rotation: Quat,
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
        ))
        .insert_resource(AgentInformation {
            position_a: Vec3::Z * 100.0,
            position_b: Vec3::Z * -100.0,
            velocity_a: Vec3::ZERO,
            velocity_b: Vec3::ZERO,
            max_velocity_a: 200.0,
            max_velocity_b: 200.0,
            half_size_a: Vec3::new(30.0, 50.0, 100.0),
            radius_b: 50.0,
            lookeahead: 5.0,
            agent_rotation: Quat::IDENTITY,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, (draw_agent_gizmos, draw_ui, draw_velocity_obstacle))
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
    ));
}

fn draw_agent_gizmos(mut gizmos: Gizmos, agent_information: Res<AgentInformation>) {
    gizmos.cuboid(
        Transform::from_translation(agent_information.position_a)
            .with_rotation(agent_information.agent_rotation)
            .with_scale(agent_information.half_size_a * 2.0),
        Color::GREEN,
    );

    gizmos.cuboid(
        Transform::from_translation(agent_information.position_a)
            .with_rotation(agent_information.agent_rotation)
            .with_scale(
                (agent_information.half_size_a + Vec3::splat(agent_information.radius_b)) * 2.0,
            ),
        Color::BLUE,
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

        ui.add(egui::Slider::new(&mut info.max_velocity_a, 0.0..=1000.0).text("(A) Max Velocity"));
        ui.add(egui::Slider::new(&mut info.max_velocity_b, 0.0..=1000.0).text("(B) Max Velocity"));

        ui.add(egui::Slider::new(&mut info.radius_b, 0.0..=100.0).text("(B) Radius"));

        ui.add(egui::Slider::new(&mut info.half_size_a.x, 0.0..=100.0).text("(A) Half Size X"));
        ui.add(egui::Slider::new(&mut info.half_size_a.y, 0.0..=100.0).text("(A) Half Size Y"));
        ui.add(egui::Slider::new(&mut info.half_size_a.z, 0.0..=100.0).text("(A) Half Size Z"));

        ui.add(egui::Slider::new(&mut info.lookeahead, 0.0..=100.0).text("Lookahead"));

        let mut euler_angles = info.agent_rotation.to_euler(EulerRot::YXZ);

        ui.add(
            egui::Slider::new(
                &mut euler_angles.0,
                -std::f32::consts::PI..=std::f32::consts::PI,
            )
            .text("Yaw"),
        );
        ui.add(
            egui::Slider::new(
                &mut euler_angles.1,
                -std::f32::consts::PI..=std::f32::consts::PI,
            )
            .text("Pitch"),
        );
        ui.add(
            egui::Slider::new(
                &mut euler_angles.2,
                -std::f32::consts::PI..=std::f32::consts::PI,
            )
            .text("Roll"),
        );

        info.agent_rotation = Quat::from_euler(
            EulerRot::YXZ,
            euler_angles.0,
            euler_angles.1,
            euler_angles.2,
        );
    });
}

fn draw_velocity_obstacle(agent_information: Res<AgentInformation>, mut gizmos: Gizmos) {
    let mut agent_self = Agent3D::new(
        agent_information.position_a,
        agent_information.velocity_a,
        Collider::new_aabb(Vec3::ZERO, agent_information.half_size_a),
    );
    agent_self.responsibility = 1.0;

    let mut agent_other = Agent3D::new(
        agent_information.position_b,
        agent_information.velocity_b,
        Collider::Sphere(Sphere::new(agent_information.radius_b, Vec3::ZERO)),
    );
    agent_other.responsibility = 0.0;

    let fvo =
        FormationVelocityObstacle3D::new(&agent_self, &agent_other, agent_information.lookeahead);

    let orca = fvo.orca_plane(0.1, 100, 100, 0.0, &mut gizmos);

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
