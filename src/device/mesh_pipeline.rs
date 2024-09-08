use std::mem;
use bytemuck::{Pod, Zeroable};
use cgmath::{Matrix4, SquareMatrix};
use is_odd::IsOdd;
use wgpu::{BindGroup, BindGroupLayout, Buffer, BufferAddress, Device, FrontFace, PipelineLayout, RenderPipeline, TextureFormat};
use wgpu::util::DeviceExt;
use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::materials::{Material, MATERIALS_COUNT};
use crate::device::{calculate_offset_pad, StepVertexBuffer};


pub const OFFSCREEN_TEXEL_SIZE: u32 = 16;
const METADATA_COUNT: u32 = 256;
const STRIGHT_COLOR: u32 = 76;
const BEND_COLOR: u32 = 37;
pub const SELECT_COLOR: u32 = 1;

pub struct MeshPipeLine {
    pub metadata: Vec<[i32; 4]>,
    pub metadata_buffer: Buffer,
    pub step_vertex_buffer: StepVertexBuffer,
    pub camera_buffer: Buffer,
    pub material_buffer: Buffer,
    pub light_buffer: Buffer,
    pub v_buffer: Buffer,
    pub i_buffer: Buffer,
    pub mesh_bind_group_layout: BindGroupLayout,
    pub mesh_render_pipeline: RenderPipeline,
    pub back_ground_pipe_line: BackGroundPipeLine,
    pub selection_render_pipeline: RenderPipeline,
    pub offscreen_width: u32,
    pub offscreen_data: Vec<i32>,
    pub offscreen_buffer: Buffer,

    pub feed_translations_state: Vec<Matrix4<f32>>,
    pub feed_translations: Vec<[f32; 16]>,
    pub feed_translations_buffer:Buffer,

}
impl MeshPipeLine {
    pub fn new(device: &Device, format: TextureFormat, w: i32, h: i32) -> Self {
        let back_ground_pipe_line = BackGroundPipeLine::new(device, format.clone(), w, h);

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
        let step_vertex_buffer: StepVertexBuffer = StepVertexBuffer::default();
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Index Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&step_vertex_buffer.indxes),
            usage: wgpu::BufferUsages::INDEX,
        });
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&step_vertex_buffer.buffer),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let mut metadata_default: Vec<[i32; 4]> = vec![];
        for i in 0..METADATA_COUNT {
            if (!i.is_odd()) {
                metadata_default.push([BEND_COLOR as i32, 0, 0, 0]);
            } else {
                metadata_default.push([STRIGHT_COLOR as i32, 0, 0, 0]);
            }
        };
        let metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&metadata_default),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let mut feed_translations_state:Vec<Matrix4<f32>> = vec![];
        for i in 0..256 {
            let mm:Matrix4<f32>=Matrix4::identity();
            feed_translations_state.push(mm.clone());
        }
        let mut feed_translations:Vec<[f32; 16]> = vec![];
        for i in 0..256 {
            let mm:Matrix4<f32>=Matrix4::identity();
            let m: &[f32; 16] = mm.as_ref();
            feed_translations.push( m.clone());
        }
        let  feed_translations_buffer: Buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("feed_translations  Buffer").as_str()),
            contents: bytemuck::cast_slice( feed_translations.as_ref()),
            usage: wgpu::BufferUsages::UNIFORM| wgpu::BufferUsages::COPY_DST
        });




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
                //Transforms
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
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[MeshVertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &mesh_shader,
                entry_point: "fs_main",
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: format,
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

        let selection_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Selection Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/selection_shader.wgsl").into()),
        });
        let selection_render_pipeline: RenderPipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Selection Render Pipeline"),
            layout: Some(&mesh_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &selection_shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[MeshVertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &selection_shader,
                entry_point: "fs_main",
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: TextureFormat::Rgba32Sint,
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

        let offscreen_width: u32 =calculate_offset_pad(w as u32);
        let mut offscreen_data: Vec<i32> = Vec::<i32>::with_capacity((offscreen_width as i32 * h * OFFSCREEN_TEXEL_SIZE as i32) as usize);
        let offscreen_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("OFSSCREEN_BUUFER"),
            size: offscreen_data.capacity() as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });




        Self {
            metadata: metadata_default,
            metadata_buffer: metadata_buffer,
            step_vertex_buffer: step_vertex_buffer,
            camera_buffer: camera_buffer,
            material_buffer: material_buffer,
            light_buffer: light_buffer,
            v_buffer: vertex_buffer,
            i_buffer: index_buffer,
            mesh_bind_group_layout: mesh_bind_group_layout,
            mesh_render_pipeline: mesh_render_pipeline,
            back_ground_pipe_line: back_ground_pipe_line,
            selection_render_pipeline: selection_render_pipeline,
            offscreen_width: offscreen_width,
            offscreen_data: offscreen_data,
            offscreen_buffer: offscreen_buffer,
            feed_translations_state:feed_translations_state,
            feed_translations:  feed_translations,
            feed_translations_buffer: feed_translations_buffer,
        }
    }
    pub fn create_bind_group(&self, device: &Device) -> BindGroup {
        let mesh_uniform_bind_group: BindGroup = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &self.mesh_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.camera_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.light_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.material_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: self.metadata_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: self.feed_translations_buffer.as_entire_binding(),
                },
            ],
            label: Some("Mesh Bind Group"),
        });
        mesh_uniform_bind_group
    }
    pub fn update_vertexes(&mut self, device: &Device) {
        self.i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Index Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&self.step_vertex_buffer.indxes),
            usage: wgpu::BufferUsages::INDEX,
        });
        self.v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&self.step_vertex_buffer.buffer),
            usage: wgpu::BufferUsages::VERTEX,
        });
    }
    pub fn select_by_id(&mut self, device: &Device, id: i32) {
        self.unselect_all();
        self.metadata[id as usize][1 as usize] = SELECT_COLOR as i32;
        self.update_meta_data(device);
        #[cfg(target_arch = "wasm32")]{
            use crate::remote::selected_by_id;
            selected_by_id(id);
        }

    }
    pub fn unselect_all(&mut self) {
        self.metadata.iter_mut().for_each(|md| {
            md[1] = 0;
        });
        #[cfg(target_arch = "wasm32")]{
            use crate::remote::selected_by_id;
            selected_by_id(0);
        }

    }
    pub fn update_meta_data(&mut self, device: &Device) {
        self.metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&self.metadata),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    }
    pub fn resize(&mut self, device: &Device, w: i32, h: i32) {
        self.offscreen_width = calculate_offset_pad(w as u32);
        self.offscreen_data = Vec::<i32>::with_capacity((self.offscreen_width as i32 * h * OFFSCREEN_TEXEL_SIZE as i32) as usize);
        self.offscreen_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("OFSSCREEN_BUUFER"),
            size: self.offscreen_data.capacity() as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 4],
    pub normal: [f32; 4],
    pub id: i32,
}

impl MeshVertex {
    pub fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0, 1.0],
            normal: [1.0, 0.0, 0.0, 1.0],
            id: 0,
        }
    }
    pub fn new(vx: f32, vy: f32, vz: f32, nx: f32, ny: f32, nz: f32, mi: i32, id: i32) -> Self {
        Self {
            position: [vx, vy, vz, 1.0],
            normal: [nx, ny, nz, 1.0],
            id: id,
        }
    }
    const ATTRIBUTES: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0=>Float32x4, 1=>Float32x4, 2=>Sint32];
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}