use bevy::{core_pipeline::clear_color::ClearColorConfig, prelude::*};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use coordination::{
    formations::{CircleFormation, LineFormation, QueueFormation, VFormation},
    Formation, FormationTemplate, FormationTemplateSet,
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
    Combined,
}

#[derive(Resource)]
struct FormationSettings {
    selected_formation: FormationType,
    circle_formation_spacing: f32,
    line_formation_spacing: f32,
    v_formation_spacing: f32,
    queue_formation_spacing: f32,

    circle_formation_priority: f32,
    line_formation_priority: f32,
    v_formation_priority: f32,
    queue_formation_priority: f32,

    combined_circle_formation_spacing: f32,
    combined_line_formation_spacing: f32,
    combined_v_formation_spacing: f32,
    combined_queue_formation_spacing: f32,

    combined_circle_formation_multiplier: f32,
    combined_line_formation_multiplier: f32,
    combined_v_formation_multiplier: f32,
    combined_queue_formation_multiplier: f32,

    obstacle_avoidance_time_horizon: f32,
    number_of_yaw_samples: u16,
    number_of_pitch_samples: u16,
    max_steps_for_em: usize,
    deformation_penalty_multiplier: f32,

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

            circle_formation_priority: 3.0,
            line_formation_priority: 12.0,
            v_formation_priority: 9.0,
            queue_formation_priority: 1.0,

            combined_circle_formation_spacing: 15.0,
            combined_line_formation_spacing: 15.0,
            combined_v_formation_spacing: 15.0,
            combined_queue_formation_spacing: 15.0,

            combined_circle_formation_multiplier: 1.0,
            combined_line_formation_multiplier: 0.0,
            combined_v_formation_multiplier: 0.0,
            combined_queue_formation_multiplier: 0.0,

            obstacle_avoidance_time_horizon: 10.0,
            number_of_yaw_samples: 50,
            number_of_pitch_samples: 50,
            max_steps_for_em: 100,
            deformation_penalty_multiplier: 0.0,
        }
    }
}

impl FormationSettings {
    fn get_formation_template(&self, formation_type: FormationType) -> Box<dyn FormationTemplate> {
        match formation_type {
            FormationType::Circle => Box::new(CircleFormation::new(
                self.agent_radius,
                self.circle_formation_spacing,
                self.circle_formation_priority,
            )),
            FormationType::Line => Box::new(LineFormation::new(
                self.agent_radius,
                self.line_formation_spacing,
                self.line_formation_priority,
            )),

            FormationType::V => Box::new(VFormation::new(
                self.agent_radius,
                self.v_formation_spacing,
                self.v_formation_priority,
            )),
            FormationType::Queue => Box::new(QueueFormation::new(
                self.agent_radius,
                self.queue_formation_spacing,
                self.queue_formation_priority,
            )),
            FormationType::Combined => Box::new(CombinedFormation {
                circle_formation: CircleFormation::new(
                    self.agent_radius,
                    self.combined_circle_formation_spacing,
                    self.circle_formation_priority,
                ),
                line_formation: LineFormation::new(
                    self.agent_radius,
                    self.combined_line_formation_spacing,
                    self.line_formation_priority,
                ),
                v_formation: VFormation::new(self.agent_radius, self.v_formation_spacing, 1.0),
                queue_formation: QueueFormation::new(
                    self.agent_radius,
                    self.combined_queue_formation_spacing,
                    self.queue_formation_priority,
                ),
                agent_radius: self.agent_radius,
                circle_formation_multiplier: self.combined_circle_formation_multiplier,
                line_formation_multiplier: self.combined_line_formation_multiplier,
                v_formation_multiplier: self.combined_v_formation_multiplier,
                queue_formation_multiplier: self.combined_queue_formation_multiplier,
            }),
        }
    }
}

struct CombinedFormation {
    circle_formation: CircleFormation,
    line_formation: LineFormation,
    v_formation: VFormation,
    queue_formation: QueueFormation,

    agent_radius: f32,
    circle_formation_multiplier: f32,
    line_formation_multiplier: f32,
    v_formation_multiplier: f32,
    queue_formation_multiplier: f32,
}

impl FormationTemplate for CombinedFormation {
    fn create_formation(&self, n_agents: usize) -> coordination::Formation {
        let circle_formation = self.circle_formation.create_formation(n_agents);
        let line_formation = self.line_formation.create_formation(n_agents);
        let v_formation = self.v_formation.create_formation(n_agents);
        let queue_formation = self.queue_formation.create_formation(n_agents);

        let mut positions = Vec::with_capacity(n_agents);
        for i in 0..n_agents {
            let circle_position = circle_formation.get_positions()[i];
            let line_position = line_formation.get_positions()[i];
            let v_position = v_formation.get_positions()[i];
            let queue_position = queue_formation.get_positions()[i];

            let position = circle_position * self.circle_formation_multiplier
                + line_position * self.line_formation_multiplier
                + v_position * self.v_formation_multiplier
                + queue_position * self.queue_formation_multiplier;

            positions.push(position);
        }

        Formation::new(positions)
    }

    fn get_priority(&self) -> f32 {
        1.0
    }

    fn get_aabb(&self, n_agents: usize) -> geometry::Aabb {
        let formation = self.create_formation(n_agents);

        formation.get_bounds(self.agent_radius)
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
            ui.radio_value(
                &mut formation_settings.selected_formation,
                FormationType::Combined,
                "Combined",
            );
        });

        match formation_settings.selected_formation {
            FormationType::Circle => draw_circle_formation_ui(ui, &mut formation_settings),
            FormationType::Line => draw_line_formation_ui(ui, &mut formation_settings),
            FormationType::V => draw_v_formation_ui(ui, &mut formation_settings),
            FormationType::Queue => draw_queue_formation_ui(ui, &mut formation_settings),
            FormationType::Combined => draw_combined_formation_ui(ui, &mut formation_settings),
        }
    });
}

fn draw_combined_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Circle Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_circle_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Line Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_line_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("V Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_v_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Queue Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_queue_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Circle Formation Multiplier");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_circle_formation_multiplier,
        0.0..=1.0,
    ));

    ui.label("Line Formation Multiplier");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_line_formation_multiplier,
        0.0..=1.0,
    ));

    ui.label("V Formation Multiplier");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_v_formation_multiplier,
        0.0..=1.0,
    ));

    ui.label("Queue Formation Multiplier");
    ui.add(egui::Slider::new(
        &mut formation_settings.combined_queue_formation_multiplier,
        0.0..=1.0,
    ));

    let sum = formation_settings.combined_circle_formation_multiplier
        + formation_settings.combined_line_formation_multiplier
        + formation_settings.combined_v_formation_multiplier
        + formation_settings.combined_queue_formation_multiplier;

    formation_settings.combined_circle_formation_multiplier /= sum;
    formation_settings.combined_line_formation_multiplier /= sum;
    formation_settings.combined_v_formation_multiplier /= sum;
    formation_settings.combined_queue_formation_multiplier /= sum;
}

fn draw_circle_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Circle Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.circle_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Circle Formation Priority");
    ui.add(egui::Slider::new(
        &mut formation_settings.circle_formation_priority,
        1.0..=100.0,
    ));
}

fn draw_queue_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Queue Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.queue_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Queue Formation Priority");
    ui.add(egui::Slider::new(
        &mut formation_settings.queue_formation_priority,
        1.0..=100.0,
    ));
}

fn draw_line_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("Line Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.line_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("Line Formation Priority");
    ui.add(egui::Slider::new(
        &mut formation_settings.line_formation_priority,
        1.0..=100.0,
    ));
}

fn draw_v_formation_ui(ui: &mut egui::Ui, formation_settings: &mut FormationSettings) {
    ui.label("V Formation Spacing");
    ui.add(egui::Slider::new(
        &mut formation_settings.v_formation_spacing,
        0.0..=100.0,
    ));

    ui.label("V Formation Priority");
    ui.add(egui::Slider::new(
        &mut formation_settings.v_formation_priority,
        1.0..=100.0,
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

    let templates = [
        formation_settings.get_formation_template(FormationType::Circle),
        formation_settings.get_formation_template(FormationType::Line),
        formation_settings.get_formation_template(FormationType::V),
        formation_settings.get_formation_template(FormationType::Queue),
    ];

    let formation_template_set = FormationTemplateSet::from_slice(&[
        templates[0].as_ref(),
        templates[1].as_ref(),
        templates[2].as_ref(),
        templates[3].as_ref(),
    ]);

    let (best_formation, best_velocity) = formation_template_set.get_best_formation_and_velocity(
        formation.get_positions(),
        Vec3::Z * 100.0,
        100.0,
        formation_settings.deformation_penalty_multiplier,
        &[],
        formation_settings.obstacle_avoidance_time_horizon,
        formation_settings.number_of_yaw_samples,
        formation_settings.number_of_pitch_samples,
        formation_settings.max_steps_for_em,
    );

    gizmos.line(aabb.center, aabb.center + best_velocity, Color::BLUE);

    for position in best_formation.get_positions() {
        gizmos.sphere(
            best_velocity + *position,
            Quat::IDENTITY,
            formation_settings.agent_radius,
            Color::BLUE,
        );
    }
}
