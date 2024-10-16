use bevy::{core_pipeline::clear_color::ClearColorConfig, prelude::*};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use coordination::{
    formations::{CircleFormation, LineFormation, QueueFormation, VFormation},
    FormationTemplate,
};
use example_utils::{
    CameraTarget, SkyboxPlugin, UniversalCamera, UniversalCameraPlugin, UtilsPlugin,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FormationType {
    Circle,
    Line,
    V,
    Queue,
}

#[derive(Resource)]
struct FormationSettings {
    selected_formation: FormationType,
    circle_formation_spacing: f32,
    line_formation_spacing: f32,
    v_formation_spacing: f32,
    queue_formation_spacing: f32,

    agent_radius: f32,
    num_agents: usize,
}

impl Default for FormationSettings {
    fn default() -> Self {
        Self {
            selected_formation: FormationType::Circle,
            num_agents: 10,
            agent_radius: 10.0,
            circle_formation_spacing: 15.0,
            line_formation_spacing: 15.0,
            v_formation_spacing: 15.0,
            queue_formation_spacing: 15.0,
        }
    }
}

impl FormationSettings {
    fn get_formation_template(&self, formation_type: FormationType) -> Box<dyn FormationTemplate> {
        match formation_type {
            FormationType::Circle => Box::new(CircleFormation::new(
                self.agent_radius,
                self.circle_formation_spacing,
                1.0,
            )),
            FormationType::Line => Box::new(LineFormation::new(
                self.agent_radius,
                self.line_formation_spacing,
                1.0,
            )),

            FormationType::V => Box::new(VFormation::new(
                self.agent_radius,
                self.v_formation_spacing,
                1.0,
            )),
            FormationType::Queue => Box::new(QueueFormation::new(
                self.agent_radius,
                self.queue_formation_spacing,
                1.0,
            )),
        }
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        resizable: false,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    file_path: "../../assets".to_string(),
                    ..Default::default()
                }),
            EguiPlugin,
            UniversalCameraPlugin,
            SkyboxPlugin,
            UtilsPlugin,
        ))
        .insert_resource(FormationSettings::default())
        .insert_resource(Msaa::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (draw_ui, draw_formations))
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3dBundle {
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..Default::default()
            },
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

fn draw_ui(mut contexts: EguiContexts, mut formation_settings: ResMut<FormationSettings>) {
    egui::Window::new("Controls").show(contexts.ctx_mut(), |ui| {
        ui.label("Number of Agents");
        ui.add(egui::Slider::new(
            &mut formation_settings.num_agents,
            1..=50,
        ));

        ui.label("Agent Radius");
        ui.add(egui::Slider::new(
            &mut formation_settings.agent_radius,
            1.0..=20.0,
        ));

        ui.separator();

        ui.horizontal(|ui| {
            ui.radio_value(
                &mut formation_settings.selected_formation,
                FormationType::Circle,
                "Circle",
            );
            ui.radio_value(
                &mut formation_settings.selected_formation,
                FormationType::Line,
                "Line",
            );
            ui.radio_value(
                &mut formation_settings.selected_formation,
                FormationType::V,
                "V",
            );
            ui.radio_value(
                &mut formation_settings.selected_formation,
                FormationType::Queue,
                "Queue",
            );
        });

        match formation_settings.selected_formation {
            FormationType::Circle => draw_circle_formation_ui(ui, &mut formation_settings),
            FormationType::Line => draw_line_formation_ui(ui, &mut formation_settings),
            FormationType::V => draw_v_formation_ui(ui, &mut formation_settings),
            FormationType::Queue => draw_queue_formation_ui(ui, &mut formation_settings),
        }
    });
}

fn draw_circle_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Circle Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.circle_formation_spacing,
        0.0..=100.0,
    ));
}

fn draw_queue_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Queue Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.queue_formation_spacing,
        0.0..=100.0,
    ));
}

fn draw_line_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Line Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.line_formation_spacing,
        0.0..=100.0,
    ));
}

fn draw_v_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("V Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.v_formation_spacing,
        0.0..=100.0,
    ));
}

fn draw_formations(mut gizmos: Gizmos, formation_settings: Res<FormationSettings>) {
    let formation_template =
        formation_settings.get_formation_template(formation_settings.selected_formation);

    let formation = formation_template.create_formation(formation_settings.num_agents);
    let aabb = formation_template.get_aabb(formation_settings.num_agents);

    for position in formation.get_positions() {
        gizmos.sphere(
            *position,
            Quat::IDENTITY,
            formation_settings.agent_radius,
            Color::GREEN,
        );
    }

    gizmos.cuboid(
        Transform::from_scale(aabb.half_sizes * 2.0).with_translation(aabb.center),
        Color::RED,
    );

    gizmos.line(aabb.center, aabb.center + Vec3::Z * 100.0, Color::RED);
}
