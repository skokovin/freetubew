use std::mem;
use bytemuck::{Pod, Zeroable};
use cgmath::{Deg, Matrix4, Point3, Rad, SquareMatrix, Vector3, Vector4};
use cgmath::num_traits::abs;
use is_odd::IsOdd;
use log::warn;
use web_sys::console::warn;
use wgpu::{BindGroup, BindGroupLayout, Buffer, BufferAddress, Device, FrontFace, PipelineLayout, RenderPipeline, TextureFormat};
use wgpu::util::DeviceExt;
use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::materials::{Material, MATERIALS_COUNT};
use crate::device::{calculate_offset_pad, StepVertexBuffer};
use crate::device::gstate::{FORWARD_DIR32, UP_DIR32};
use crate::trialgo::pathfinder::{CncOps, OpElem, LRACMD};

pub const OFFSCREEN_TEXEL_SIZE: u32 = 16;
const METADATA_COUNT: u32 = 256;
const STRIGHT_COLOR: u32 = 76;
const BEND_COLOR: u32 = 37;
pub const SELECT_COLOR: u32 = 1;

pub struct MeshPipeLine {
    pub metadata: Vec<[i32; 4]>,
    pub metadata_buffer: Buffer,
    pub step_vertex_buffer: Vec<StepVertexBuffer>,
    pub camera_buffer: Buffer,
    pub material_buffer: Buffer,
    pub light_buffer: Buffer,
    pub v_buffer: Vec<Buffer>,
    pub i_buffer: Vec<Buffer>,
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

    pub ops:CncOps,

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
        //let step_vertex_buffer: StepVertexBuffer = StepVertexBuffer::default();
        let step_vertex_buffer  :Vec<StepVertexBuffer> = vec![];

        let index_buffer:Vec<Buffer> = vec![];
        let vertex_buffer :Vec<Buffer> = vec![];
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
            ops: CncOps::default(),
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
        self.i_buffer =vec![];
        self.v_buffer =vec![];

        let mut index=0;
        self.step_vertex_buffer.iter().for_each(|item|{
            let i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Index Mesh Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.indxes),
                usage: wgpu::BufferUsages::INDEX,
            });
            let v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Vertex Mesh Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.buffer),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.i_buffer.push(i_buffer);
            self.v_buffer.push(v_buffer);
            index=index+1;
        });

        println!("{:?}",self.v_buffer.len());

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

    pub fn init_model(&mut self, device: &Device, buff:Vec<StepVertexBuffer>){
        self.step_vertex_buffer=buff;
        self.update_vertexes(device);

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

    pub fn do_step(&mut self,step:usize,t_dorn:Matrix4<f32>, r_dorn:Matrix4<f32>,t_feed:Matrix4<f32>,r_feed:Matrix4<f32>) {
        for i in (0..step) {
            self.feed_translations_state[i]=t_dorn*r_dorn*r_feed*self.feed_translations_state[i];
        }
        for i in (step)..256 {
            self.feed_translations_state[i]=self.feed_translations_state[i]*t_feed;
        }
        self.update_transformations();
    }

    pub fn calculate_bend_step(&mut self,device: &Device){


        let op: LRACMD =self.ops.do_bend();


        match op.op_code {
            0 => {
                let r_deg_angle = Deg(op.value1 as f32);
                let feed_tr: Matrix4<f32> = Matrix4::from_translation(Vector3::<f32>::new(op.value0 as f32, 0.0, 0.0));
                let feed_r: Matrix4<f32> =Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(r_deg_angle));
                let dorn_tr: Matrix4<f32> = Matrix4::from_translation(Vector3::<f32>::new(op.value0 as f32, 0.0, 0.0));
                let dorn_r: Matrix4<f32> = Matrix4::identity();

                self.do_step(
                    op.id as usize,
                    dorn_tr,
                    dorn_r,
                    feed_tr,
                    feed_r,
                );
            }
            1 => {

                let offset=self.ops.unbend_offsets[op.id as usize];
                let dorn_radius: f32 = op.value1 as f32;
                let deg_angle = Deg(op.value0 as f32);
                let bend_radius=dorn_radius+op.pipe_radius as f32;
                //let deg_angle = Deg(30.0);
                let dorn_move_scalar = abs(Rad::from(deg_angle).0 * bend_radius);

                let r_feed: Matrix4<f32> = Matrix4::identity();
                let t_feed: Matrix4<f32> = Matrix4::from_translation(Vector3::<f32>::new(dorn_move_scalar, 0.0, 0.0));

                let p0: Point3<f32> = Point3::new(0.0, 0.0, 0.0);
                let cp: Point3<f32> = Point3::new(0.0, -bend_radius, 0.0);
                let v = p0 - cp;
                let r_dorn: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad::from(deg_angle));
                //let r_dorn: Matrix4<f32> = Matrix4::identity();
                let rotated: Vector4<f32> = r_dorn * v.extend(1.0);
                let new_vec: Vector3<f32> = Vector3::new(-rotated.x, (bend_radius - rotated.y), 0.0);
                warn!("new_vec_a {:?} ",self.step_vertex_buffer.len());
                //let new_vec: Vector3<f32> = Vector3::new(45.0, 12.0, 0.0);
                let t_dorn: Matrix4<f32> = Matrix4::from_translation(new_vec);

                let new_buff: StepVertexBuffer =CncOps::generate_tor_by_cnc(&op, offset as f64);

                self.step_vertex_buffer[op.id as usize]=new_buff;

                self.update_vertexes(device);
                self.do_step(
                    op.id as usize,
                    t_dorn,
                    r_dorn,
                    t_feed,
                    r_feed,
                );

            }
            _ => {}
        }
    }

    pub fn update_transformations(&mut self) {
        for i in 0..256 {
            let m_ft: &[f32; 16] =self.feed_translations_state[i].as_ref();
            self.feed_translations[i]=(m_ft.clone());
        }
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