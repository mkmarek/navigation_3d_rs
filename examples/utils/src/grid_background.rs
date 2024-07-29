use bevy::{
    prelude::*,
    render::{
        mesh::shape::Quad,
        render_resource::{AsBindGroup, ShaderRef},
    },
    sprite::{Material2d, MaterialMesh2dBundle, Mesh2dHandle},
};

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
pub struct GridTexture {
    #[uniform(0)]
    pub scale: Vec2,

    #[uniform(1)]
    pub position_offset: Vec2,

    #[uniform(2)]
    pub resolution: Vec2,
}

impl Material for GridTexture {
    fn fragment_shader() -> ShaderRef {
        "shaders/grid_texture.wgsl".into()
    }
}

impl Material2d for GridTexture {
    fn fragment_shader() -> ShaderRef {
        "shaders/grid_texture.wgsl".into()
    }
}

pub fn spawn_grid(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut grid_texture_materials: ResMut<Assets<GridTexture>>,
) {
    commands.spawn(MaterialMesh2dBundle {
        mesh: Mesh2dHandle(meshes.add(Quad::default().into())),
        transform: Transform::default()
            .with_translation(Vec3::new(0.0, 0.0, -10.0))
            .with_scale(Vec3::splat(128.)),
        material: grid_texture_materials.add(GridTexture {
            scale: Vec2::splat(1.0),
            position_offset: Vec2::ZERO,
            resolution: Vec2::splat(128.0),
        }),
        ..default()
    });
}

pub fn update_grid_texture_materials(
    mut materials: ResMut<Assets<GridTexture>>,
    camera_query: Query<(&GlobalTransform, &Camera)>,
    mut query: Query<(&Handle<GridTexture>, &mut Transform)>,
) {
    if camera_query.is_empty() {
        return;
    }

    let (camera_transform, camera) = camera_query.single();

    for (grid_texture_handle, mut transform) in query.iter_mut() {
        let viewport = camera.logical_viewport_size().unwrap();
        let top_left = camera
            .viewport_to_world_2d(camera_transform, Vec2::new(0.0, viewport.y))
            .unwrap();
        let right_bottom = camera
            .viewport_to_world_2d(camera_transform, Vec2::new(viewport.x, 0.0))
            .unwrap();
        let grid_texture = materials.get_mut(grid_texture_handle).unwrap();
        let camera_world_size = (right_bottom - top_left).abs();

        grid_texture.resolution = viewport;
        grid_texture.scale = camera_world_size;
        grid_texture.position_offset = top_left;
        let camera_translation = camera_transform.translation().truncate();
        transform.translation = Vec3::new(
            camera_translation.x,
            camera_translation.y,
            transform.translation.z,
        );
        transform.scale = Vec3::new(camera_world_size.x, camera_world_size.y, 1.0);
    }
}
