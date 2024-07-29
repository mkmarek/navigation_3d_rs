use std::f32::consts::PI;

use bevy::{
    input::mouse::MouseWheel,
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};

pub enum CameraTarget {
    Entity(Entity),
    Position(Vec3),
}

#[derive(Component)]
pub enum UniversalCamera {
    Orbit {
        focus: CameraTarget,
        offset: Vec3,
        current_focus: Vec3,
        radius: f32,
        locked_cursor_position: Option<Vec2>,
    },
    FirstPerson {
        translation_speed: f32,
        look_sensitivity: f32,
        locked_cursor_position: Option<Vec2>,
    },
}

impl Default for UniversalCamera {
    fn default() -> Self {
        UniversalCamera::FirstPerson {
            translation_speed: 10.0,
            look_sensitivity: 0.0002,
            locked_cursor_position: None,
        }
    }
}

pub struct UniversalCameraPlugin;

impl Plugin for UniversalCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            (
                grab_and_release_mouse,
                universal_camera_movement.after(grab_and_release_mouse),
                universal_camera_look.after(universal_camera_movement),
                update_camera.after(universal_camera_look),
            ),
        );
    }
}

fn grab_and_release_mouse(
    state: Res<Input<MouseButton>>,
    mut windows: Query<&mut Window>,
    mut cameras: Query<&mut UniversalCamera>,
) {
    let camera = cameras.get_single_mut();

    if camera.is_err() {
        return;
    }

    let mut camera = camera.unwrap();

    for mut window in windows.iter_mut() {
        if state.just_pressed(MouseButton::Right) {
            window.cursor.grab_mode = CursorGrabMode::Locked;
            window.cursor.visible = false;

            if let UniversalCamera::Orbit {
                locked_cursor_position,
                ..
            } = camera.as_mut()
            {
                *locked_cursor_position = window.cursor_position();
            } else if let UniversalCamera::FirstPerson {
                locked_cursor_position,
                ..
            } = camera.as_mut()
            {
                *locked_cursor_position = window.cursor_position();
            }
        }

        if state.just_released(MouseButton::Right) {
            window.cursor.grab_mode = CursorGrabMode::None;
            window.cursor.visible = true;
        }
    }
}

fn update_camera(
    transforms: Query<&Transform, Without<UniversalCamera>>,
    mut cameras: Query<(&mut UniversalCamera, &mut Transform, &Projection)>,
    time: Res<Time>,
) {
    cameras.for_each_mut(|(mut camera, mut transform, _)| match camera.as_mut() {
        UniversalCamera::FirstPerson {
            translation_speed: _,
            look_sensitivity: _,
            locked_cursor_position: _,
        } => {}
        UniversalCamera::Orbit {
            focus,
            offset,
            current_focus,
            radius,
            locked_cursor_position: _,
        } => {
            let focus_target = match focus {
                CameraTarget::Entity(entity) => {
                    if let Ok(entity_transform) = transforms.get(*entity) {
                        entity_transform.translation
                    } else {
                        *current_focus
                    }
                }
                CameraTarget::Position(position) => *position,
            };

            *current_focus = Vec3::lerp(*current_focus, focus_target, time.delta_seconds() * 10.0);

            let rot_matrix = Mat3::from_quat(transform.rotation);
            transform.translation =
                *current_focus + rot_matrix.mul_vec3(Vec3::new(0.0, 0.0, *radius));

            let up = transform.rotation * Vec3::Y;
            let upside_down = up.y <= 0.0;
            transform.look_at(
                *current_focus,
                if upside_down { Vec3::NEG_Y } else { Vec3::Y },
            );

            transform.translation += rot_matrix.mul_vec3(*offset * *radius);
        }
    });
}

fn universal_camera_look(
    mut cameras: Query<(&mut UniversalCamera, &mut Transform, &Projection)>,
    primary_window: Query<&Window, With<PrimaryWindow>>,
) {
    if let Ok(window) = primary_window.get_single() {
        if window.cursor.grab_mode == CursorGrabMode::None {
            return;
        }

        let mouse_position = window.cursor_position();

        if mouse_position.is_none() {
            return;
        }

        let mouse_position = mouse_position.unwrap();

        cameras.for_each_mut(
            move |(mut camera, mut transform, _)| match camera.as_mut() {
                UniversalCamera::FirstPerson {
                    translation_speed: _,
                    look_sensitivity,
                    locked_cursor_position,
                } => {
                    let delta = mouse_position - locked_cursor_position.unwrap();
                    locked_cursor_position.replace(mouse_position);
                    let window_scale = window.height().min(window.width());
                    let pitch = -(*look_sensitivity * delta.y * window_scale).to_radians();
                    let yaw = -(*look_sensitivity * delta.x * window_scale).to_radians();

                    transform.rotation = transform.rotation
                        * Quat::from_axis_angle(Vec3::Y, yaw)
                        * Quat::from_axis_angle(Vec3::X, pitch);
                }
                UniversalCamera::Orbit {
                    offset: _,
                    focus: _,
                    current_focus: _,
                    radius: _,
                    locked_cursor_position,
                } => {
                    let delta = mouse_position - locked_cursor_position.unwrap();
                    locked_cursor_position.replace(mouse_position);

                    if delta.length_squared() > 0.0 {
                        let window = Vec2::new(window.width(), window.height());
                        let up = transform.rotation * Vec3::Y;
                        let upside_down = up.y <= 0.0;

                        let delta_x = {
                            let delta = delta.x / window.x * PI * 2.0;
                            if upside_down {
                                -delta
                            } else {
                                delta
                            }
                        };

                        let delta_y = delta.y / window.y * PI;
                        let yaw = Quat::from_rotation_y(-delta_x);
                        let pitch = Quat::from_rotation_x(-delta_y);

                        transform.rotation = yaw * transform.rotation; // rotate around global y axis
                        transform.rotation *= pitch; // rotate around local x axis
                    }
                }
            },
        )
    }
}

fn universal_camera_movement(
    mut cameras: Query<(&mut UniversalCamera, &mut Transform, &Projection)>,
    keys: Res<Input<KeyCode>>,
    primary_window: Query<&Window, With<PrimaryWindow>>,
    time: Res<Time>,
    mut ev_scroll: EventReader<MouseWheel>,
) {
    if let Ok(window) = primary_window.get_single() {
        cameras.for_each_mut(
            move |(mut camera, mut transform, _)| match camera.as_mut() {
                UniversalCamera::FirstPerson {
                    translation_speed,
                    look_sensitivity: _,
                    locked_cursor_position: _,
                } => {
                    if window.cursor.grab_mode != CursorGrabMode::None {
                        let mut velocity = Vec3::ZERO;
                        let forward = -transform.local_z();
                        let right = transform.local_x();
                        let up = transform.local_y();

                        for key in keys.get_pressed() {
                            let key = *key;
                            if key == KeyCode::W {
                                velocity += forward;
                            } else if key == KeyCode::S {
                                velocity -= forward;
                            } else if key == KeyCode::A {
                                velocity -= right;
                            } else if key == KeyCode::D {
                                velocity += right;
                            } else if key == KeyCode::Q {
                                velocity -= up;
                            } else if key == KeyCode::E {
                                velocity += up;
                            }
                        }

                        transform.translation +=
                            velocity * time.delta_seconds() * *translation_speed
                    }
                }
                UniversalCamera::Orbit {
                    focus: _,
                    offset: _,
                    current_focus: _,
                    radius,
                    locked_cursor_position: _,
                } => {
                    for event in ev_scroll.read() {
                        *radius *= if event.y.signum() < 0.0 {
                            1.0 / 1.2
                        } else {
                            1.2
                        }
                    }
                }
            },
        );
    }
}
