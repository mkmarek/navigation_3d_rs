use bevy::{
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderRef},
};

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone, Default)]
pub struct PlaneMaterial {
    #[uniform(0)]
    color: Color,

    #[uniform(1)]
    number_of_lines: f32,
}

impl PlaneMaterial {
    pub fn new(color: Color, number_of_lines: f32) -> Self {
        Self {
            color,
            number_of_lines,
        }
    }
}

impl Material for PlaneMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/plane_material_shader.wgsl".into()
    }

    fn specialize(
        _pipeline: &bevy::pbr::MaterialPipeline<Self>,
        descriptor: &mut bevy::render::render_resource::RenderPipelineDescriptor,
        _layout: &bevy::render::mesh::MeshVertexBufferLayout,
        _key: bevy::pbr::MaterialPipelineKey<Self>,
    ) -> Result<(), bevy::render::render_resource::SpecializedMeshPipelineError> {
        descriptor.primitive.cull_mode = None;

        Ok(())
    }
}
