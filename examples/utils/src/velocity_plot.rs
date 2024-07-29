use bevy::{
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderRef},
    sprite::Material2d,
};

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
pub struct VelocityTexture {
    #[uniform(0)]
    pub in_collision_color: Color,
    #[uniform(1)]
    pub out_of_collision_color: Color,

    #[uniform(2)]
    pub radius_sum: f32,

    #[uniform(3)]
    pub first_agent_position: Vec2,

    #[uniform(4)]
    pub second_agent_position: Vec2,

    #[uniform(5)]
    pub second_agent_velocity: Vec2,

    #[uniform(6)]
    pub look_ahead_time: f32,

    #[uniform(7)]
    pub resolution: Vec2,
}

impl Material2d for VelocityTexture {
    fn fragment_shader() -> ShaderRef {
        "shaders/velocity_texture.wgsl".into()
    }
}
