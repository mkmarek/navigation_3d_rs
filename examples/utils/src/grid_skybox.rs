use bevy::{
    prelude::{App, FromWorld, Plugin, QueryState, Resource, World},
    render::{
        color::Color,
        main_graph::node::CAMERA_DRIVER,
        mesh::PrimitiveTopology,
        render_graph::{Node, NodeRunError, RenderGraph, RenderGraphContext},
        render_resource::{
            BindGroupEntry, BindGroupLayout, BindGroupLayoutDescriptor, BindGroupLayoutEntry,
            BindingType, BlendState, BufferBindingType, ColorTargetState, ColorWrites, Face,
            FrontFace, LoadOp, MultisampleState, Operations, PipelineLayoutDescriptor, PolygonMode,
            PrimitiveState, RawFragmentState, RawRenderPipelineDescriptor, RawVertexState,
            RenderPassDescriptor, RenderPipeline, ShaderModuleDescriptor, ShaderSource,
            ShaderStages, TextureFormat, UniformBuffer,
        },
        renderer::{RenderContext, RenderDevice, RenderQueue},
        view::{ExtractedView, ViewTarget},
        RenderApp,
    },
};

pub struct SkyboxPlugin;

impl Plugin for SkyboxPlugin {
    fn finish(&self, app: &mut App) {
        let render_app = app.sub_app_mut(RenderApp);

        render_app.init_resource::<RenderSkyboxPipeline>();

        let render_skybox_node = RenderSkyboxNode::new(&mut render_app.world);

        let mut graph = render_app.world.get_resource_mut::<RenderGraph>().unwrap();
        graph.add_node(RENDER_SKYBOX_NODE, render_skybox_node);
        graph.add_node_edge(RENDER_SKYBOX_NODE, CAMERA_DRIVER);
    }

    fn build(&self, _app: &mut App) {}
}

pub const RENDER_SKYBOX_NODE: &str = "render_skybox";

pub struct RenderSkyboxNode {
    query: QueryState<(&'static ViewTarget, &'static ExtractedView)>,
}

impl RenderSkyboxNode {
    pub fn new(world: &mut World) -> Self {
        Self {
            query: QueryState::new(world),
        }
    }
}

impl Node for RenderSkyboxNode {
    fn update(&mut self, world: &mut World) {
        self.query.update_archetypes(world);
    }
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let render_queue = world.get_resource::<RenderQueue>().unwrap();
        let pipeline = world.get_resource::<RenderSkyboxPipeline>().unwrap();

        for (camera_target, view) in self.query.iter_manual(world) {
            let mut projection_buffer = UniformBuffer::from(view.projection.inverse());
            let mut view_buffer = UniformBuffer::from(view.transform.compute_matrix());

            projection_buffer.write_buffer(render_context.render_device(), render_queue);
            view_buffer.write_buffer(render_context.render_device(), render_queue);

            let texture_bind_group = render_context.render_device().create_bind_group(
                "Skybox Texture Bind Group",
                &pipeline.source_texture_bing_group_layout,
                &[
                    BindGroupEntry {
                        binding: 0,
                        resource: projection_buffer.binding().unwrap(),
                    },
                    BindGroupEntry {
                        binding: 1,
                        resource: view_buffer.binding().unwrap(),
                    },
                ],
            );

            let mut render_pass =
                render_context
                    .command_encoder()
                    .begin_render_pass(&RenderPassDescriptor {
                        label: Some("Render skybox to Camera Pass"),
                        color_attachments: &[Some(camera_target.get_color_attachment(
                            Operations {
                                load: LoadOp::Clear(Color::rgba(0.0, 0.0, 0.0, 1.0).into()),
                                store: true,
                            },
                        ))],
                        depth_stencil_attachment: None,
                    });

            render_pass.set_bind_group(0, &texture_bind_group, &[]);
            render_pass.set_pipeline(&pipeline.render_pipeline);
            render_pass.draw(0..4, 0..1);
        }

        Ok(())
    }
}

#[derive(Resource)]
pub struct RenderSkyboxPipeline {
    pub render_pipeline: RenderPipeline,
    pub source_texture_bing_group_layout: BindGroupLayout,
}

impl FromWorld for RenderSkyboxPipeline {
    fn from_world(world: &mut bevy::prelude::World) -> Self {
        let device = world.get_resource::<RenderDevice>().unwrap();
        let shader = device.create_shader_module(ShaderModuleDescriptor {
            label: Some("Shader"),
            source: ShaderSource::Wgsl(
                include_str!("../../../assets/shaders/grid_skybox.wgsl").into(),
            ),
        });

        let source_texture_bing_group_layout =
            device.create_bind_group_layout(&BindGroupLayoutDescriptor {
                label: Some("Camera Bind Group Layout"),
                entries: &[
                    BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::FRAGMENT,
                        ty: BindingType::Buffer {
                            ty: BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    BindGroupLayoutEntry {
                        binding: 1,
                        visibility: ShaderStages::FRAGMENT,
                        ty: BindingType::Buffer {
                            ty: BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            });

        let render_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("Skybox pipeline Layout"),
            bind_group_layouts: &[&source_texture_bing_group_layout],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&RawRenderPipelineDescriptor {
            label: Some("Skybox pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: RawVertexState {
                module: &shader,
                entry_point: "vert",
                buffers: &[],
            },
            fragment: Some(RawFragmentState {
                module: &shader,
                entry_point: "frag",
                targets: &[Some(ColorTargetState {
                    format: TextureFormat::Rgba8UnormSrgb,
                    blend: Some(BlendState::ALPHA_BLENDING),
                    write_mask: ColorWrites::ALL,
                })],
            }),
            primitive: PrimitiveState {
                topology: PrimitiveTopology::TriangleStrip,
                strip_index_format: None,
                front_face: FrontFace::Ccw,
                cull_mode: Some(Face::Back),
                polygon_mode: PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: MultisampleState {
                count: 4,
                mask: !0,
                alpha_to_coverage_enabled: true,
            },
            // If the pipeline will be used with a multiview render pass, this
            // indicates how many array layers the attachments will have.
            multiview: None,
        });

        Self {
            render_pipeline,
            source_texture_bing_group_layout,
        }
    }
}
