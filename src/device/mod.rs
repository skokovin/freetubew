use std::mem;
use std::ops::Sub;
use bytemuck::{Pod, Zeroable};
use cgmath::{Point3, Vector3};
use serde::{Deserialize, Serialize};
use truck_base::bounding_box::BoundingBox;
use wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;

mod camera;
mod mesh_pipeline;
pub(crate) mod materials;
pub mod aux_state;
pub mod background_pipleine;
pub mod gstate;

pub mod scene;


pub const Z_FIGHTING_FACTOR: f32 = 0.0001;
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Triangle {
    pub p: [Point3<f32>; 3],
    pub normal: Vector3<f32>,
}
impl Triangle {
    pub fn new(p0: Point3<f32>, p1: Point3<f32>, p2: Point3<f32>) -> Self {
        let u = p1.sub(p0);
        let v = p2.sub(p0);
        let normal: Vector3<f32> = Vector3::new(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        );
        Self {
            p: [p0, p1, p2],
            normal: normal,
        }
    }

    pub fn fromX64(
        p0: truck_base::cgmath64::Point3,
        p1: truck_base::cgmath64::Point3,
        p2: truck_base::cgmath64::Point3,
        n: truck_base::cgmath64::Vector3,
    ) -> Self {
        let normal: Vector3<f32> = cgmath::Vector3::new(n.x as f32, n.y as f32, n.z as f32);
        Self {
            p: [
                Point3::new(p0.x as f32, p0.y as f32, p0.z as f32),
                Point3::new(p1.x as f32, p1.y as f32, p1.z as f32),
                Point3::new(p2.x as f32, p2.y as f32, p2.z as f32)],
            normal: normal,
        }
    }


    pub fn from_coords(
        x0: f32, y0: f32, z0: f32,
        x1: f32, y1: f32, z1: f32,
        x2: f32, y2: f32, z2: f32,
    ) -> Self {
        let p0: Point3<f32> = Point3::new(x0, y0, z0);
        let p1: Point3<f32> = Point3::new(x1, y1, z1);
        let p2: Point3<f32> = Point3::new(x2, y2, z2);
        Triangle::new(p0, p1, p2)
    }

    pub fn as_p64(&self) -> [truck_base::cgmath64::Point3; 3] {
        let p64: [truck_base::cgmath64::Point3; 3] = [
            Point3::new(self.p[0].x as f64, self.p[0].y as f64, self.p[0].z as f64),
            Point3::new(self.p[1].x as f64, self.p[1].y as f64, self.p[1].z as f64),
            Point3::new(self.p[2].x as f64, self.p[2].y as f64, self.p[2].z as f64)
        ];
        p64
    }
}

#[derive(Clone)]
pub struct RawMesh {
    pub id: i32,
    pub vertex_normal: Vec<f32>,
    pub indx: Vec<i32>,
    pub bbx: BoundingBox<Point3<f64>>,
    pub triangles: Vec<Triangle>,
}
impl Default for RawMesh {
    fn default() -> Self {
        RawMesh {
            id: -999,
            vertex_normal: vec![],
            indx: vec![],
            bbx: BoundingBox::default(),
            triangles: vec![],
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
    pub fn new(vx: f32, vy: f32, vz: f32, nx: f32, ny: f32, nz: f32, id: i32) -> Self {
        Self {
            position: [vx, vy, vz, 1.0],
            normal: [nx, ny, nz, 1.0],
            id: id,
        }
    }
    const ATTRIBUTES: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0=>Float32x4, 1=>Float32x4,  3=>Sint32];
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<MeshVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}


pub struct StepVertexBuffer {
    pub buffer: Vec<MeshVertex>,
    pub indxes: Vec<u32>,
    pub id_hash: Vec<u32>,
}

impl StepVertexBuffer {
    pub fn update(&mut self, buffer: Vec<MeshVertex>, indxes: Vec<u32>, id_hash: Vec<u32>) {
        self.buffer = buffer;
        self.indxes = indxes;
        self.id_hash = id_hash;
    }
    pub fn default() -> Self {
        StepVertexBuffer {
            buffer: vec![],
            indxes: vec![],
            id_hash: vec![],
        }
    }
    pub fn clean(&mut self) {
        self.buffer = vec![];
        self.indxes = vec![];
        self.id_hash = vec![];
    }
}


pub fn calculate_offset_pad(curr_mem:u32)->u32{
    if(curr_mem<COPY_BYTES_PER_ROW_ALIGNMENT){
        COPY_BYTES_PER_ROW_ALIGNMENT
    }else{
        let mut count=curr_mem/COPY_BYTES_PER_ROW_ALIGNMENT;
        if(curr_mem%COPY_BYTES_PER_ROW_ALIGNMENT !=0){
            count=count+1;
        }
        let memcount=count*COPY_BYTES_PER_ROW_ALIGNMENT;
        memcount
    }
}

