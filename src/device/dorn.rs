use cgmath::{Matrix4, Point3, Vector3};
use wgpu::{Buffer, Device};
use wgpu::util::DeviceExt;
use crate::device::StepVertexBuffer;
use crate::trialgo::bendpipe::MainCylinder;

const DORN_THICKNESS: f32 = 20.0;
pub const DORN_PARK_POSITION: f32 = 110.0;
const DORN_UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const DORN_RIGHT: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
pub const DORN_ID: usize = 250;
pub const DORN_DEFAULT_RADIUS: f32 = 20.0;
pub struct Dorn {
    pub v_buffer: Vec<Buffer>,
    pub i_buffer: Vec<Buffer>,
    pub step_vertex_buffer: Vec<StepVertexBuffer>,
    pub park_translation: Matrix4<f32>,
}

impl Dorn {
    pub fn default() -> Self {
        let mut d: Dorn = Dorn {
            v_buffer: vec![],
            i_buffer: vec![],
            step_vertex_buffer: vec![],
            park_translation: Matrix4::from_translation(Vector3::<f32>::new(0.0, DORN_PARK_POSITION, 0.0)),
        };
        d.set_dorn(DORN_DEFAULT_RADIUS);
        d
    }

    pub fn set_dorn(&mut self, dorn_radius: f32) {
        let mc = MainCylinder::by_2points_r(
            Point3::new(0.0, 0.0, DORN_THICKNESS as f64),
            Point3::new(0.0, 0.0, -DORN_THICKNESS as f64), dorn_radius as f64, DORN_ID as u32);
        self.step_vertex_buffer=vec![];
        self.step_vertex_buffer.push(mc);
    }

    pub fn update_vertexes(&mut self, device: &Device) {
        self.i_buffer = vec![];
        self.v_buffer = vec![];
        let mut index = 0;
        self.step_vertex_buffer.iter().for_each(|item| {
            let i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Index Dorn Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.indxes),
                usage: wgpu::BufferUsages::INDEX,
            });
            let v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Vertex Dorn Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.buffer),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.i_buffer.push(i_buffer);
            self.v_buffer.push(v_buffer);
            index = index + 1;
        });
    }

}