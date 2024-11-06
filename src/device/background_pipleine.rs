use wgpu::{BindGroup, BindGroupLayout, Buffer, Device, PipelineLayout, RenderPipeline, TextureFormat};
use wgpu::util::DeviceExt;

pub struct BackGroundPipeLine {
    pub background_bind_group_layout: BindGroupLayout,
    pub render_pipeline: RenderPipeline,
    pub add_data_buffer: Buffer,
    pub mesh_uniform_bind_group: BindGroup,
}
impl BackGroundPipeLine {
    pub fn new(device: &Device, format: &TextureFormat,w:i32,h:i32) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Mesh Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/background_shader.wgsl").into()),
        });
        let arr:[i32;4]=[w,h,0,0];
        let add_data_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("AddData Uniform Buffer").as_str()),
            contents: bytemuck::cast_slice(&arr),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let background_bind_group_layout: BindGroupLayout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },

            ],
            label: Some("mesh Bind Group Layout"),
        });
        let pipeline_layout: PipelineLayout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&background_bind_group_layout],
            push_constant_ranges: &[],
        });
        let render_pipeline: RenderPipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: None,
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(format.clone().into())],
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let mesh_uniform_bind_group: BindGroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &background_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: add_data_buffer.as_entire_binding(),
                },

            ],
            label: Some("Mesh Bind Group"),
        });
        Self {
            background_bind_group_layout: background_bind_group_layout,
            render_pipeline: render_pipeline,
            add_data_buffer:add_data_buffer,
            mesh_uniform_bind_group:mesh_uniform_bind_group,
        }
    }

}