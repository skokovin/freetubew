use cgmath::{Matrix4, Point3, Vector3};
use wgpu::{Buffer, Device};
use wgpu::util::DeviceExt;
use crate::device::mesh_pipeline::{DORN_AXIS_Y_ID, DORN_AXIS_Y_MOVED_ID, DORN_AXIS_Z_ID, DORN_AXIS_Z_MOVED_ID, DORN_ID};
use crate::device::StepVertexBuffer;

use crate::trialgo::bendpipe::MainCylinder;


const DORN_THICKNESS: f32 = 20.0;
pub const DORN_PARK_POSITION: f32 = 210.0;
const DORN_UP: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
const DORN_RIGHT: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);


pub const DORN_DEFAULT_RADIUS: f32 = 20.0;
pub const AXIS_Y_THICKNESS: f64 = 1.5;
pub const AXIS_Z_THICKNESS: f64 = 1.7;
pub const SEMI_LENGTH: f64 = 300.0;

pub const DORN_AXIS_Y_COLOR: usize = 89;
pub const DORN_AXIS_Z_COLOR: usize = 10;

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
        let dorn_symbol = MainCylinder::by_2points_r(
            Point3::new(0.0, 0.0, DORN_THICKNESS as f64),
            Point3::new(0.0, 0.0, -DORN_THICKNESS as f64), dorn_radius as f64, DORN_ID as u32);
        let dorn_z_symbol = MainCylinder::by_2points_r(
            Point3::new(0.0, 0.0, (DORN_THICKNESS + 20.0) as f64),
            Point3::new(0.0, 0.0, -(DORN_THICKNESS + 20.0) as f64), AXIS_Y_THICKNESS, DORN_ID as u32);

        let mc_y = MainCylinder::by_2points_r(
            Point3::new(0.0, -SEMI_LENGTH, 0.0),
            Point3::new(0.0, SEMI_LENGTH, 0.0), AXIS_Y_THICKNESS, DORN_AXIS_Y_ID as u32);
        let mc_y_moved = MainCylinder::by_2points_r(
            Point3::new(0.0, -SEMI_LENGTH, 0.0),
            Point3::new(0.0, SEMI_LENGTH, 0.0), AXIS_Y_THICKNESS, DORN_AXIS_Y_MOVED_ID as u32);

        let mc_z = MainCylinder::by_2points_r(
            Point3::new(0.0, 0.0, -SEMI_LENGTH),
            Point3::new(0.0, 0.0, SEMI_LENGTH), AXIS_Z_THICKNESS, DORN_AXIS_Z_ID as u32);
        let mc_z_moved = MainCylinder::by_2points_r(
            Point3::new(0.0, 0.0, -SEMI_LENGTH),
            Point3::new(0.0, 0.0, SEMI_LENGTH), AXIS_Z_THICKNESS, DORN_AXIS_Z_MOVED_ID as u32);


        self.step_vertex_buffer = vec![];
        self.step_vertex_buffer.extend(dorn_symbol);
        self.step_vertex_buffer.extend(dorn_z_symbol);
        self.step_vertex_buffer.extend(mc_y);
        self.step_vertex_buffer.extend(mc_y_moved);
        self.step_vertex_buffer.extend(mc_z);
        self.step_vertex_buffer.extend(mc_z_moved);
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