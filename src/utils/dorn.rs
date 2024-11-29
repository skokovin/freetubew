use std::ops::Mul;
use cgmath::{Matrix4, Point3, SquareMatrix, Vector3};
use log::warn;
use rand::random;
use shipyard::{Unique};
use truck_base::bounding_box::BoundingBox;
use wgpu::{BindingResource, Buffer, Device};
use wgpu::util::DeviceExt;
use crate::algo::{round_by_dec, MainCircle, MainCylinder, DIVIDER, P_FORWARD, P_RIGHT, P_UP_REVERSE};
use crate::algo::cnc::LRACLR;
use crate::device::{StepVertexBuffer};
use crate::device::graphics::AnimState;
use crate::utils::dim::DimB;

const DORN_THICKNESS: f32 = 200.0;
const DORN_ID: u64 = 200;
const DORN_RADIUS: f64 = 20.0;
const DORN_HEIGHT: f64 = 20.0;
const DORN_PARK_POSITION: f64 = 300.0;
#[derive(Unique)]
pub struct Dorn {
    pub is_scale_dirty: bool,
    pub is_translate_dirty: bool,
    pub scale: Matrix4<f32>,
    pub translate: Matrix4<f32>,
    pub v_buffer: Buffer,
    pub i_buffer: Buffer,
    pub scale_buffer: Buffer,
    pub translate_buffer: Buffer,
}

impl Dorn {
    pub fn new(device: &Device, v_up_orign: &Vector3<f64>) -> Dorn {
        let mut m_scale: Matrix4<f32> = Matrix4::identity();
        m_scale.x[0] = DORN_RADIUS as f32;
        m_scale.y[1] = DORN_RADIUS as f32;
        m_scale.z[2] = DORN_HEIGHT as f32;
        let m: &[f32; 16] = m_scale.as_ref();
        let scale_buffer: Buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("dorn scale Buffer").as_str()),
            contents: bytemuck::cast_slice(m.as_ref()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let mut m_translate: Matrix4<f32> = Matrix4::from_translation(
            Vector3::<f32>::new(P_RIGHT.x as f32,P_RIGHT.y as f32,P_RIGHT.z as f32).mul(DORN_PARK_POSITION as f32 * v_up_orign.z as f32)
        );
        let mt: &[f32; 16] = m_translate.as_ref();
        let translate_buffer: Buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("dorn translate Buffer").as_str()),
            contents: bytemuck::cast_slice(mt.as_ref()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let ca = MainCircle {
            id: random(),
            radius: 1.0,
            loc: Point3::new(0.0, 0.0, 0.5),
            dir: P_UP_REVERSE,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(1.0, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: random(),
            radius: 1.0,
            loc: Point3::new(0.0, 0.0, -0.5),
            dir: P_UP_REVERSE,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(1.0, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: DORN_ID,
            ca: ca,
            cb: cb,
            h: 1.0,
            r: 1.0,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        mc.triangulate();
        let (v_buffer, i_buffer) = mc.step_vertex_buffer.to_buffers(device);

        Self {
            is_scale_dirty: true,
            is_translate_dirty: true,
            scale: m_scale,
            translate: m_translate,
            v_buffer: v_buffer,
            i_buffer: i_buffer,
            scale_buffer: scale_buffer,
            translate_buffer: translate_buffer,
        }
    }

    pub fn update(&mut self, device: &Device) {
        if (self.is_scale_dirty) {
            let m: &[f32; 16] = self.scale.as_ref();
            self.scale_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("dorn scale Buffer").as_str()),
                contents: bytemuck::cast_slice(m.as_ref()),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            self.is_scale_dirty = false;
        }
        if (self.is_translate_dirty) {
            let m: &[f32; 16] = self.translate.as_ref();
            self.translate_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("dorn scale Buffer").as_str()),
                contents: bytemuck::cast_slice(m.as_ref()),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            self.is_translate_dirty = false;
        }
    }

    pub fn update_dorn_radius(&mut self, radius: f64) {
        if (self.scale.x[0] != radius as f32) {
            self.scale.x[0] = radius as f32;
            self.scale.y[1] = radius as f32;
            self.is_scale_dirty = true;
        }
    }

    pub fn update_dorn_height(&mut self, h: f64) {
        if (self.scale.z[2] != h as f32) {
            self.scale.z[2] = h as f32;
            self.is_scale_dirty = true;
        }
    }

    pub fn update_dorn_position(&mut self, y: f64, v_up_orign: &Vector3<f64>) {
        self.translate = Matrix4::from_translation(
            Vector3::<f32>::new(P_RIGHT.x as f32,P_RIGHT.y as f32,P_RIGHT.z as f32).mul(y as f32 * v_up_orign.z as f32)
        );
        self.is_translate_dirty = true;
    }

    pub fn set_dorn_park(&mut self,v_up_orign: &Vector3<f64>) {
        
        self.translate = Matrix4::from_translation(
            Vector3::<f32>::new(P_RIGHT.x as f32,P_RIGHT.y as f32,P_RIGHT.z as f32).mul(DORN_PARK_POSITION as f32 * v_up_orign.z as f32)
        );
        self.is_translate_dirty = true;
    }

    pub fn dorn_action(&mut self, anim_state: &AnimState, v_up_orign: &Vector3<f64>) {
        match anim_state.opcode {
            2 => {
                self.update_dorn_height(anim_state.lra.pipe_radius * 2.0);
                self.update_dorn_radius(anim_state.lra.clr - anim_state.lra.pipe_radius);
                self.update_dorn_position(anim_state.lra.clr,v_up_orign);
            }
            _ => {
                self.set_dorn_park(v_up_orign);
            }
        }
    }
}
unsafe impl Send for Dorn {}
unsafe impl Sync for Dorn {}