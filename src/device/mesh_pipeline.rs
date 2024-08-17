use std::mem;
use bytemuck::{Pod, Zeroable};
use is_odd::IsOdd;
use wgpu::{BindGroup, BindGroupLayout, Buffer, BufferAddress, Device, FrontFace, PipelineLayout, RenderPipeline, TextureFormat};
use wgpu::util::DeviceExt;
use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::materials::{Material, MATERIALS_COUNT};
use crate::device::StepVertexBuffer;

const METADATA_COUNT:u32=256;
const STRIGHT_COLOR:u32=76;
const BEND_COLOR:u32=37;
pub const SELECT_COLOR:u32=1;

pub struct MeshPipeLine {
    pub metadata:Vec<[i32;4]>,
    pub metadata_buffer:Buffer,
    pub step_vertex_buffer: StepVertexBuffer,
    pub camera_buffer: Buffer,
    pub material_buffer: Buffer,
    pub light_buffer: Buffer,
    pub v_buffer: Buffer,
    pub i_buffer: Buffer,
    pub mesh_bind_group_layout: BindGroupLayout,
    pub mesh_render_pipeline: RenderPipeline,
    pub back_ground_pipe_line: BackGroundPipeLine,
}
impl MeshPipeLine {
    pub fn new(device: &Device, format: TextureFormat,w:i32,h:i32) -> Self {
        let back_ground_pipe_line=BackGroundPipeLine::new(device,format.clone(),w,h);

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
        let mut metadata_default:Vec<[i32;4]>=vec![];
        for i in 0..METADATA_COUNT {
            if(i.is_odd()){
                metadata_default.push([BEND_COLOR as i32, 0,0,0]);
            }else{
                metadata_default.push([STRIGHT_COLOR as i32, 0,0,0]);
            }

        };
        let metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&metadata_default),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
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



        Self {
            metadata:metadata_default,
            metadata_buffer:metadata_buffer,
            step_vertex_buffer: step_vertex_buffer,
            camera_buffer: camera_buffer,
            material_buffer: material_buffer,
            light_buffer: light_buffer,
            v_buffer: vertex_buffer,
            i_buffer: index_buffer,
            mesh_bind_group_layout: mesh_bind_group_layout,
            mesh_render_pipeline: mesh_render_pipeline,
            back_ground_pipe_line:back_ground_pipe_line,
        }
    }
    pub fn create_bind_group(&self,device: &Device) -> BindGroup {
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
            ],
            label: Some("Mesh Bind Group"),
        });
        mesh_uniform_bind_group
    }
    pub fn update_vertexes(&mut self,device: &Device) {
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
    pub fn select_by_id(&mut self,device: &Device, id:i32){
        self.unselect_all();
        self.metadata[id as usize ][1 as usize]= SELECT_COLOR as i32;
        self.update_meta_data(device);
    }
    pub fn unselect_all(&mut self){
        self.metadata.iter_mut().for_each(|md|{
            md[1]=0;
        });
    }
    fn update_meta_data(&mut self,device: &Device){
        self.metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&self.metadata),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
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