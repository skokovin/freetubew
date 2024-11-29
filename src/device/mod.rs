use std::mem;
use bytemuck::{Pod, Zeroable};
use wgpu::{Buffer, Device};
use wgpu::util::DeviceExt;

pub mod graphics;
pub mod background_pipleine;
pub mod bend_pipe;
pub mod camera;
pub mod mesh_pipeline;

pub mod app;
pub mod txt_pipeline;

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable,Default)]
pub struct MeshVertex {
    pub position: [f32; 4],
    pub normal: [f32; 4],
    pub id: i32,
    pub tex_coords: [f32; 2],
}

impl MeshVertex {
    pub fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0, 1.0],
            normal: [1.0, 0.0, 0.0, 1.0],
            id: 0,
            tex_coords: [0.0,0.0],
        }
    }
    pub fn new(vx: f32, vy: f32, vz: f32, nx: f32, ny: f32, nz: f32, mi: i32, id: i32,tex_u:f32,tex_v:f32,) -> Self {
        Self {
            position: [vx, vy, vz, 1.0],
            normal: [nx, ny, nz, 1.0],
            id: id,
            tex_coords: [tex_u,tex_v],
        }
    }
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![0=>Float32x4, 1=>Float32x4, 2=>Sint32, 3=>Float32x2];
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

#[repr(C)]
#[derive(Clone, Debug,Default)]
pub struct StepVertexBuffer {
    pub buffer: Vec<MeshVertex>,
    pub indxes: Vec<u32>,
    // pub id_hash: Vec<u32>,
}
impl StepVertexBuffer {
    pub fn update(&mut self, buffer: Vec<MeshVertex>, indxes: Vec<u32>) {
        self.buffer = buffer;
        self.indxes = indxes;
    }
    pub fn default() -> Self {
        StepVertexBuffer {
            buffer: vec![],
            indxes: vec![],
        }
    }
    pub fn clean(&mut self) {
        self.buffer = vec![];
        self.indxes = vec![];
    }
    pub fn to_buffers(&self, device: &Device) -> (Buffer, Buffer) {
        let index = self.buffer[0].id;

        let i_buffer: Buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Index Mesh Buffer {index}").as_str()),
            contents: bytemuck::cast_slice(&self.indxes),
            usage: wgpu::BufferUsages::INDEX,
        });
        let v_buffer: Buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer {index}").as_str()),
            contents: bytemuck::cast_slice(&self.buffer),
            usage: wgpu::BufferUsages::VERTEX,
        });
        (v_buffer, i_buffer)
    }
}



