use cgmath::{Matrix4, SquareMatrix};
use is_odd::IsOdd;
use shipyard::UniqueViewMut;
use wgpu::{BindGroup, BindGroupLayout, Buffer, BufferAddress, Device, FrontFace, PipelineLayout, RenderPipeline, TextureFormat};
use wgpu::util::DeviceExt;
use crate::device::{MeshVertex, StepVertexBuffer};
use crate::device::graphics::Graphics;
use crate::utils::materials::{Material, MATERIALS_COUNT};
const METADATA_COUNT: usize = 256;
const STRIGHT_COLOR: u32 = 76;
const BEND_COLOR: u32 = 37;
pub struct MeshPipeLine {
    pub camera_buffer: Buffer,
    pub material_buffer: Buffer,
    pub materials: Vec<Material>,
    pub light_buffer: Buffer,
    pub v_buffer: Vec<Buffer>,
    pub i_buffer: Vec<Buffer>,
    pub mesh_bind_group_layout: BindGroupLayout,
    pub mesh_render_pipeline: RenderPipeline,
    pub metadata_buffer: Buffer,

}

impl MeshPipeLine {
    pub fn new(device: &Device, format: &TextureFormat, w: i32, h: i32) -> Self {
        let camera_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Camera Uniform Buffer"),
            size: 144,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let material_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Material Uniform Buffer"),
            size: (size_of::<Material>() * MATERIALS_COUNT) as BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let light_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Light Uniform Buffer"),
            size: 48,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let index_buffer: Vec<Buffer> = vec![];
        let vertex_buffer: Vec<Buffer> = vec![];
        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Mesh Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/mesh_shader.wgsl").into()),
        });
        let mesh_bind_group_layout: BindGroupLayout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                //Camera
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
                //CameraUniforms
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                //LightUniforms
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                //MetaData
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                //DornScale
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                //DornTranslate
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
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
        let mesh_pipeline_layout: PipelineLayout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Mesh Render Pipeline Layout"),
            bind_group_layouts: &[&mesh_bind_group_layout],
            push_constant_ranges: &[],
        });
        let mesh_render_pipeline: RenderPipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Mesh Render Pipeline"),
            layout: Some(&mesh_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &mesh_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[MeshVertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &mesh_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: format.clone(),
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: FrontFace::default(),
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: Default::default(),
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let mut metadata_default: Vec<[i32; 4]> = vec![];
        for i in 0..METADATA_COUNT {
            if (!i.is_odd()) {
                metadata_default.push([STRIGHT_COLOR as i32, 0, 0, 0]);
            } else {
                metadata_default.push([BEND_COLOR as i32, 0, 0, 0]);
            }
        };
        let metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&metadata_default),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });


        Self {
            camera_buffer: camera_buffer,
            material_buffer: material_buffer,
            materials: Material::generate_materials(),
            light_buffer: light_buffer,
            v_buffer: vertex_buffer,
            i_buffer: index_buffer,
            mesh_bind_group_layout: mesh_bind_group_layout,
            mesh_render_pipeline: mesh_render_pipeline,
            metadata_buffer:metadata_buffer,
        }
    }

}

pub fn update_mesh_render_vertexes(steps_data:&Vec<StepVertexBuffer>, graphics: &mut UniqueViewMut<Graphics>){
    graphics.mesh_pipe_line.i_buffer = vec![];
    graphics.mesh_pipe_line.v_buffer = vec![];
    let mut index = 0;
    steps_data.iter().for_each(|item| {
        let i_buffer: Buffer =graphics.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Index Mesh Buffer {index}").as_str()),
            contents: bytemuck::cast_slice(&item.indxes),
            usage: wgpu::BufferUsages::INDEX,
        });
        let v_buffer: Buffer = graphics.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer {index}").as_str()),
            contents: bytemuck::cast_slice(&item.buffer),
            usage: wgpu::BufferUsages::VERTEX,
        });
        graphics.mesh_pipe_line.i_buffer.push(i_buffer);
        graphics.mesh_pipe_line.v_buffer.push(v_buffer);
        index = index + 1;
    });
}