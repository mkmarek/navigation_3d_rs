use bevy::{prelude::*, sprite::Material2dPlugin};

mod grid_background;
mod plane_material;
mod universal_camera;
mod velocity_plot;

pub use grid_background::GridTexture;
pub use plane_material::PlaneMaterial;
pub use universal_camera::CameraTarget;
pub use universal_camera::UniversalCamera;
pub use universal_camera::UniversalCameraPlugin;
pub use velocity_plot::VelocityTexture;

pub struct UtilsPlugin;

impl Plugin for UtilsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            Material2dPlugin::<VelocityTexture>::default(),
            Material2dPlugin::<GridTexture>::default(),
            MaterialPlugin::<PlaneMaterial>::default(),
        ));

        app.add_systems(Startup, grid_background::spawn_grid);
        app.add_systems(Update, grid_background::update_grid_texture_materials);
    }
}
