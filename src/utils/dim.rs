
use crate::algo::{ round_by_dec, BendToro, MainCircle, MainCylinder, DIVIDER, P_FORWARD, P_FORWARD_REVERSE, P_RIGHT, P_RIGHT_REVERSE, P_UP, P_UP_REVERSE, ROT_DIR_CCW, TESS_TOL_ANGLE};
use crate::device::{MeshVertex, StepVertexBuffer};
use cgmath::num_traits::{abs, signum};
use cgmath::{
    Basis3, Deg, InnerSpace, Matrix4, Point3, Rad, Rotation, Rotation3, SquareMatrix, Vector3,
};
use image::{DynamicImage, Rgba, RgbaImage};
use log::warn;
use rand::random;
use rusttype::{point, Font, Scale};
use shipyard::Unique;
use std::f64::consts::PI;
use std::mem;
use std::ops::{Mul, Sub};
use truck_base::bounding_box::BoundingBox;
use truck_geometry::prelude::Plane;
use wgpu::{Buffer, BufferAddress, Device, Queue, Sampler, Texture, TextureView};

const DIM_X_THICKNESS: f64 = 3.0;
const DIM_REF_L_RADIUS: f64 = 1.5;
const REF_LINE_L_FACTOR: f64 = 20.0;
const REF_LINE_OFFSET_FACTOR: f64 = 1.25;
const ROTATION_STEP_ANGLE: f64 = 1.0;
const DIM_X_ID: i32 = 201;
const DIM_Z_ID: i32 = 202;
const DIM_B_ID: i32 = 203;
const FONT_ROBOTO_REGULAR: &[u8; 171676] = include_bytes!("../files/Roboto-Regular.ttf");

const BUFFER_SIZE_LIMIT: usize = 16590 * 3;
const ZEROS_I: [i32;BUFFER_SIZE_LIMIT] = [0;BUFFER_SIZE_LIMIT];
const ZEROS_F: [f32;BUFFER_SIZE_LIMIT] = [0.0;BUFFER_SIZE_LIMIT];

const TXT_SIZE_W: u32 = 880;
const TXT_SIZE_H: u32 = 340;
const TXT_H_RADIUS_FACTOR: f64 = 12.5;

#[derive(Unique)]
pub struct DimX {
    pub is_dirty: bool,
    pub is_active: bool,
    pub pipe_radius: f64,
    pub pos_x: f64,
    pub pos_y: f64,
    pub scale: f64,
    pub v_buffer_x: Buffer,
    pub i_buffer_x: Buffer,
    pub v_txt_buffer: Buffer,
    pub diffuse_texture: Texture,
    pub diffuse_texture_view: TextureView,
    pub diffuse_sampler: Sampler,
}
impl DimX {
    pub fn new(device: &Device) -> DimX {
        let diffuse_texture: Texture = device.create_texture(&wgpu::TextureDescriptor {
            // All textures are stored as 3D, we represent our 2D texture
            // by setting depth to 1.
            size: wgpu::Extent3d {
                width: TXT_SIZE_W,
                height: TXT_SIZE_H,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1, // We'll talk about this a little later
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            // Most images are stored using sRGB, so we need to reflect that here.
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            // TEXTURE_BINDING tells wgpu that we want to use this texture in shaders
            // COPY_DST means that we want to copy data to this texture
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: Some("diffuse_texture"),
            // This is the same as with the SurfaceConfig. It
            // specifies what texture formats can be used to
            // create TextureViews for this texture. The base
            // texture format (Rgba8UnormSrgb in this case) is
            // always supported. Note that using a different
            // texture format is not supported on the WebGL2
            // backend.
            view_formats: &[],
        });
        let diffuse_texture_view: TextureView =
            diffuse_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let diffuse_sampler: Sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let v_txt_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DimX TXT Buffer"),
            size: (TXT_SIZE_W * TXT_SIZE_H * mem::size_of::<f32>() as u32) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let i_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Arrow1 Mesh I Buffer"),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<i32>()) as BufferAddress,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let v_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(format!("Arrow1 Mesh V Buffer {DIM_X_ID}").as_str()),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<MeshVertex>()) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            is_dirty: false,
            is_active: false,
            pipe_radius: 0.0,
            pos_x: 0.0,
            pos_y: 0.0,
            scale: 1.0,
            v_buffer_x: v_buffer,
            i_buffer_x: i_buffer,
            v_txt_buffer: v_txt_buffer,
            diffuse_texture: diffuse_texture,
            diffuse_texture_view: diffuse_texture_view,
            diffuse_sampler: diffuse_sampler,
        }
    }
    pub fn update(&mut self, device: &Device, queue: &Queue, v_up_orign: &Vector3<f64>) {
        if (self.is_dirty) {
            let mut i: Vec<u32> = vec![];
            let mut v: Vec<MeshVertex> = vec![];
            let mut index: u32 = 0;

            let (arr1_v, arr1_i, arr1_last_index) = DimX::generate_arrow_x(
                DIM_X_ID,
                1.0,
                self.scale,
                self.pos_x,
                self.pipe_radius,
                v_up_orign,
                index,
            );
            index = arr1_last_index;
            i.extend_from_slice(arr1_i.as_slice());
            v.extend_from_slice(arr1_v.as_slice());

            let (arr2_v, arr2_i, arr2_last_index) = DimX::generate_arrow_x(
                DIM_X_ID,
                -1.0,
                self.scale,
                0.0,
                self.pipe_radius,
                v_up_orign,
                index,
            );
            index = arr2_last_index;
            i.extend_from_slice(arr2_i.as_slice());
            v.extend_from_slice(arr2_v.as_slice());

            let (rl1, last_index) =
                DimX::generate_line_y_x(DIM_X_ID, 0.0, self.pipe_radius, v_up_orign, index); //-DIM_X_THICKNESS as f64
            index = last_index;
            i.extend_from_slice(rl1.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl1.step_vertex_buffer.buffer.as_slice());

            let (rl2, last_index) =
                DimX::generate_line_y_x(DIM_X_ID, self.pos_x, self.pipe_radius, v_up_orign, index); //-DIM_X_THICKNESS as f64
            index = last_index;
            i.extend_from_slice(rl2.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl2.step_vertex_buffer.buffer.as_slice());

            let (rl3, last_index) =
                DimX::generate_line_x_x(DIM_X_ID, self.pos_x, self.pipe_radius, v_up_orign, index);
            index = last_index;
            i.extend_from_slice(rl3.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl3.step_vertex_buffer.buffer.as_slice());
            
            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&ZEROS_I));
            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&i));
            queue.write_buffer(&self.v_buffer_x, 0, bytemuck::cast_slice(&v));

            //TXT LAYOUT
            {
                let l = calc_ref_len_offset(self.pipe_radius) * v_up_orign.z;
                let len = l as f32 - 40.0;
                let offsetx: f32 = -20.0;
                let offsetx1: f32 = -20.0 - TXT_SIZE_W as f32;
                let mut v_txt: Vec<MeshVertex> = vec![];
                let ma0 = MeshVertex {
                    position: [offsetx, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma1 = MeshVertex {
                    position: [offsetx, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 0.0],
                };
                let ma2 = MeshVertex {
                    position: [offsetx1, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma3 = MeshVertex {
                    position: [offsetx, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma4 = MeshVertex {
                    position: [offsetx1, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma5 = MeshVertex {
                    position: [offsetx1, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 1.0],
                };
                v_txt.push(ma0);
                v_txt.push(ma1);
                v_txt.push(ma2);
                v_txt.push(ma3);
                v_txt.push(ma4);
                v_txt.push(ma5);
                self.update_glyphs(queue);
                queue.write_buffer(&self.v_txt_buffer, 0, bytemuck::cast_slice(&v_txt));
            }
            self.is_dirty = false
        }
    }
    fn update_glyphs(&mut self, queue: &Queue) {
        let diffuse_rgba = update_txt(self.pos_x, self.pipe_radius);
        queue.write_texture(
            // Tells wgpu where to copy the pixel data
            wgpu::ImageCopyTexture {
                texture: &self.diffuse_texture,
                mip_level: 0,
                origin: wgpu::Origin3d {
                    x: 0,
                    y: TXT_SIZE_H - diffuse_rgba.dimensions().1,
                    z: 0,
                },
                aspect: wgpu::TextureAspect::All,
            },
            // The actual pixel data
            &diffuse_rgba,
            // The layout of the texture
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * diffuse_rgba.dimensions().0),
                rows_per_image: Some(diffuse_rgba.dimensions().1),
            },
            wgpu::Extent3d {
                width: diffuse_rgba.dimensions().0,
                height: diffuse_rgba.dimensions().1,
                depth_or_array_layers: 1,
            },
        );
    }
    pub fn set_pipe_radius(&mut self, pipe_radius: f64) {
        self.pipe_radius = pipe_radius;
        self.is_dirty = true;
    }
    pub fn set_y(&mut self, y: f64) {
        self.pos_y = y;
        self.is_dirty = true;
    }
    pub fn set_x(&mut self, x: f64) {
        self.pos_x = x;
        self.is_dirty = true;
    }
    pub fn set_scale(&mut self, scale: f64) {
        self.scale = scale;
        self.is_dirty = true;
    }
    fn generate_arrow_x(
        id: i32,
        dir: f64,
        scale: f64,
        x: f64,
        pipe_radius: f64,
        v_up_orign: &Vector3<f64>,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let thik = DIM_X_THICKNESS as f32;
        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let len = calc_ref_len_offset(pipe_radius) * v_up_orign.z;
        let cp = Point3::new(x, -len, 0.0);
        let r_arrow_up = Point3::new(cp.x + 3.0 * scale * dir, cp.y + 0.8038 * scale * dir, 0.0);
        let r_arrow_down = Point3::new(cp.x + 3.0 * scale * dir, cp.y - 0.8038 * scale * dir, 0.0);

        let n1: [f32; 4] = [0.0, 0.0, P_UP.z as f32, 0.0];
        let n2: [f32; 4] = [0.0, 0.0, P_UP_REVERSE.z as f32, 0.0];
        let n_b: [f32; 4] = [(P_FORWARD.x * dir) as f32, 0.0, 0.0, 0.0];
        let fwd_dir: Vector3<f64> = P_UP;
        let n1_v = fwd_dir.cross(r_arrow_up.sub(cp).normalize());
        let n2_v = r_arrow_down.sub(cp).normalize().cross(fwd_dir);
        let n_a: [f32; 4] = [n1_v.x as f32, n1_v.y as f32, n1_v.z as f32, 0.0];
        let n_c: [f32; 4] = [n2_v.x as f32, n2_v.y as f32, n2_v.z as f32, 0.0];

        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }

        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }

        {
            let mv0 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        (buffer, indxes, index)
    }
    fn generate_line_x_x(
        id: i32,
        x: f64,
        pipe_radius: f64,
        v_up_orign: &Vector3<f64>,
        last_index: u32,
    ) -> (MainCylinder, u32) {
        let len = calc_ref_len_offset(pipe_radius) * v_up_orign.z;
        let ca = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: Point3::new(0.0, -len, 0.0),
            dir: P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: Point3::new(x, -len, 0.0),
            dir: P_FORWARD,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: abs(x),
            r: DIM_REF_L_RADIUS,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        let last_index = mc.triangulate_with_start_index_no_cap(last_index);
        (mc, last_index)
    }
    fn generate_line_y_x(
        id: i32,
        x: f64,
        pipe_radius: f64,
        v_up_orign: &Vector3<f64>,
        last_index: u32,
    ) -> (MainCylinder, u32) {
        let len = calc_ref_len(pipe_radius) * v_up_orign.z;

        let ca = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: Point3::new(x, 0.0, 0.0),
            dir: P_RIGHT_REVERSE,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: Point3::new(x, -len, 0.0),
            dir: P_RIGHT_REVERSE,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: abs(len),
            r: DIM_REF_L_RADIUS,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        let last_index = mc.triangulate_with_start_index_no_cap(last_index);
        (mc, last_index)
    }
}
unsafe impl Send for DimX {}
unsafe impl Sync for DimX {}

#[derive(Unique)]
pub struct DimZ {
    pub is_dirty: bool,
    pub is_active: bool,
    pub pipe_radius: f64,
    pub pos_z: f64,
    pub r: f64,
    pub scale: f64,
    pub v_buffer_x: Buffer,
    pub i_buffer_x: Buffer,
    pub v_txt_buffer: Buffer,
    pub diffuse_texture: Texture,
    pub diffuse_texture_view: TextureView,
    pub diffuse_sampler: Sampler,
}
impl DimZ {
    pub fn new(device: &Device) -> DimZ {
        let diffuse_texture: Texture = device.create_texture(&wgpu::TextureDescriptor {
            // All textures are stored as 3D, we represent our 2D texture
            // by setting depth to 1.
            size: wgpu::Extent3d {
                width: TXT_SIZE_W,
                height: TXT_SIZE_H,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1, // We'll talk about this a little later
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            // Most images are stored using sRGB, so we need to reflect that here.
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            // TEXTURE_BINDING tells wgpu that we want to use this texture in shaders
            // COPY_DST means that we want to copy data to this texture
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: Some("diffuse_texture"),
            // This is the same as with the SurfaceConfig. It
            // specifies what texture formats can be used to
            // create TextureViews for this texture. The base
            // texture format (Rgba8UnormSrgb in this case) is
            // always supported. Note that using a different
            // texture format is not supported on the WebGL2
            // backend.
            view_formats: &[],
        });
        let diffuse_texture_view: TextureView =
            diffuse_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let diffuse_sampler: Sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let v_txt_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DimX TXT Buffer"),
            size: (TXT_SIZE_W * TXT_SIZE_H * mem::size_of::<f32>() as u32) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let i_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Arrow1 Mesh I Buffer"),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<i32>()) as BufferAddress,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let v_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(format!("Arrow1 Mesh V Buffer {DIM_X_ID}").as_str()),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<MeshVertex>()) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            is_dirty: false,
            is_active: false,
            pipe_radius: 0.0,
            pos_z: 0.0,
            r: 0.0,
            scale: 1.0,
            v_buffer_x: v_buffer,
            i_buffer_x: i_buffer,
            v_txt_buffer: v_txt_buffer,
            diffuse_texture: diffuse_texture,
            diffuse_texture_view: diffuse_texture_view,
            diffuse_sampler: diffuse_sampler,
        }
    }
    pub fn update(&mut self, device: &Device, queue: &Queue) {
        if (self.is_dirty) {
            let mut i: Vec<u32> = vec![];
            let mut v: Vec<MeshVertex> = vec![];
            let mut index: u32 = 0;

            let (rl1, last_index) = DimZ::generate_line_z_y(DIM_Z_ID, 0.0, self.pipe_radius, index);
            index = last_index;
            i.extend_from_slice(rl1.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl1.step_vertex_buffer.buffer.as_slice());

            let (rl2, last_index) =
                DimZ::generate_line_z_y(DIM_Z_ID, self.r, self.pipe_radius, index);
            index = last_index;
            i.extend_from_slice(rl2.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl2.step_vertex_buffer.buffer.as_slice());

            let (arcv, arci, last_index) =
                DimZ::generate_y_arc(DIM_Z_ID, self.r, self.scale, self.pipe_radius, index);

            index = last_index;
            i.extend_from_slice(arci.as_slice());
            v.extend_from_slice(arcv.as_slice());
            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&ZEROS_I));
            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&i));
            queue.write_buffer(&self.v_buffer_x, 0, bytemuck::cast_slice(&v));
            //TXT LAYOUT
            {
                let len = calc_ref_len_offset(self.pipe_radius) as f32 - 40.0;
                let offsetx: f32 = 20.0;
                let offsetx1: f32 = 20.0 + TXT_SIZE_W as f32;
                let mut v_txt: Vec<MeshVertex> = vec![];
                let ma0 = MeshVertex {
                    position: [0.0, offsetx, len, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma1 = MeshVertex {
                    position: [0.0, offsetx, (TXT_SIZE_H as f32 + len), 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 0.0],
                };
                let ma2 = MeshVertex {
                    position: [0.0, offsetx1, (TXT_SIZE_H as f32 + len), 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma3 = MeshVertex {
                    position: [0.0, offsetx, len, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma4 = MeshVertex {
                    position: [0.0, offsetx1, (TXT_SIZE_H as f32 + len), 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma5 = MeshVertex {
                    position: [0.0, offsetx1, len, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 1.0],
                };
                v_txt.push(ma0);
                v_txt.push(ma1);
                v_txt.push(ma2);
                v_txt.push(ma3);
                v_txt.push(ma4);
                v_txt.push(ma5);
                self.update_glyphs(queue);
                queue.write_buffer(&self.v_txt_buffer, 0, bytemuck::cast_slice(&v_txt));
            }
            self.is_dirty = false;
        }
    }

    fn update_glyphs(&mut self, queue: &Queue) {
        let diffuse_rgba = update_txt(self.r, self.pipe_radius);
        queue.write_texture(
            // Tells wgpu where to copy the pixel data
            wgpu::ImageCopyTexture {
                texture: &self.diffuse_texture,
                mip_level: 0,
                origin: wgpu::Origin3d {
                    x: 0,
                    y: TXT_SIZE_H - diffuse_rgba.dimensions().1,
                    z: 0,
                },
                aspect: wgpu::TextureAspect::All,
            },
            // The actual pixel data
            &diffuse_rgba,
            // The layout of the texture
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * diffuse_rgba.dimensions().0),
                rows_per_image: Some(diffuse_rgba.dimensions().1),
            },
            wgpu::Extent3d {
                width: diffuse_rgba.dimensions().0,
                height: diffuse_rgba.dimensions().1,
                depth_or_array_layers: 1,
            },
        );
    }

    pub fn set_pipe_radius(&mut self, pipe_radius: f64) {
        self.pipe_radius = pipe_radius;
        self.is_dirty = true;
    }
    pub fn set_z(&mut self, z: f64) {
        self.pos_z = z;
        self.is_dirty = true;
    }
    pub fn set_r(&mut self, r: f64) {
        self.r = r;
        self.is_dirty = true;
    }
    pub fn set_scale(&mut self, scale: f64) {
        self.scale = scale;
        self.is_dirty = true;
    }
    fn generate_y_arc(
        id: i32,
        r_deg: f64,
        arrow_scale: f64,
        pipe_radius: f64,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let len = calc_ref_len_offset(pipe_radius);
        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];

        let r = Rad::from(Deg(r_deg));
        let cp: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

        let mut ref_line_a_dir: Vector3<f64> = P_UP;
        let mut pa: Point3<f64> = cp + ref_line_a_dir.mul(len);
        let mut a_dir: Vector3<f64> = P_RIGHT.mul(signum(r_deg));

        let rot_axe: Vector3<f64> = P_FORWARD.mul(signum(r_deg * ROT_DIR_CCW));
        let step = Rad::from(Deg(ROTATION_STEP_ANGLE));
        let deg_step = abs((r.0 / step.0) as i64);
        let mut b_dir: Vector3<f64> = a_dir;

        for i in 0..deg_step {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(rot_axe, step);
            let ref_line_b_dir = rotation.rotate_vector(ref_line_a_dir);
            let pb: Point3<f64> = cp + ref_line_b_dir.mul(len);
            b_dir = ref_line_b_dir.cross(rot_axe);

            let ca = MainCircle {
                id: random(),
                radius: DIM_REF_L_RADIUS,
                loc: pa,
                dir: a_dir,
                radius_dir: P_FORWARD,
                r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
            };
            let cb = MainCircle {
                id: random(),
                radius: DIM_REF_L_RADIUS,
                loc: pb,
                dir: b_dir,
                radius_dir: P_FORWARD,
                r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
            };

            let arr_a: Vec<Point3<f64>> = ca.gen_points();
            let arr_b: Vec<Point3<f64>> = cb.gen_points();

            for i in 0..arr_a.iter().len() - 1 {
                let p0 = arr_a[i];
                let p1 = arr_a[i + 1];
                let p2 = arr_b[i];
                let p3 = arr_b[i + 1];

                let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
                let n = plane.normal().normalize();
                let mut n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
                let r_dir = p0.sub(ca.loc);
                let is_coplanar = n.dot(r_dir);
                if (is_coplanar < 0.0) {
                    n_arr = [-n_arr[0], -n_arr[1], -n_arr[2], 0.0];
                }

                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            }
            ref_line_a_dir = ref_line_b_dir;
            pa = pb;
            a_dir = b_dir;
        }

        let parrow1 = cp + P_UP.mul(len);
        let (buffer_a1, indxes_a1, last_index_a1) = DimZ::generate_arrow_a_y(
            id,
            P_RIGHT.mul(signum(r_deg * ROT_DIR_CCW)),
            arrow_scale,
            parrow1,
            index,
        );
        index = last_index_a1;
        indxes.extend_from_slice(indxes_a1.as_slice());
        buffer.extend_from_slice(buffer_a1.as_slice());

        let (buffer_a2, indxes_a2, last_index_a2) =
            DimZ::generate_arrow_b_y(id, b_dir.mul(-1.0), arrow_scale, pa, index);
        index = last_index_a2;

        indxes.extend_from_slice(indxes_a2.as_slice());
        buffer.extend_from_slice(buffer_a2.as_slice());

        (buffer, indxes, index)
    }
    fn generate_line_z_y(
        id: i32,
        r_deg: f64,
        pipe_radius: f64,
        last_index: u32,
    ) -> (MainCylinder, u32) {
        let r = Rad::from(Deg(r_deg));
        let len = calc_ref_len(pipe_radius);

        let rotation: Basis3<f64> = Rotation3::from_axis_angle(P_FORWARD_REVERSE, r);
        let dir_z = rotation.rotate_vector(P_UP);
        let pe: Point3<f64> = Point3::new(0.0, 0.0, 0.0) + dir_z.mul(len);

        let ca = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: Point3::new(0.0, 0.0, 0.0),
            dir: P_UP,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: pe,
            dir: P_UP,
            radius_dir: P_FORWARD,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: abs(len),
            r: DIM_REF_L_RADIUS,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        let last_index = mc.triangulate_with_start_index_no_cap(last_index);
        (mc, last_index)
    }
    fn generate_arrow_a_y(
        id: i32,
        dir: Vector3<f64>,
        scale: f64,
        position: Point3<f64>,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let thik = DIM_X_THICKNESS as f32;
        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let fwd_dir = P_FORWARD;
        let up_dir = dir.cross(fwd_dir);

        let cp = position;
        let tmp_p = cp + 3.0 * scale * dir;

        let r_arrow_up = tmp_p + 0.8038 * scale * up_dir;
        let r_arrow_down = tmp_p - 0.8038 * scale * up_dir;

        let n1: [f32; 4] = [1.0, 0.0, 0.0, 0.0];
        let n2: [f32; 4] = [-1.0, 0.0, 0.0, 0.0];

        let n1_v = r_arrow_up.sub(cp).normalize().cross(fwd_dir);
        let n2_v = fwd_dir.cross(r_arrow_down.sub(cp).normalize());
        let n_a: [f32; 4] = [n1_v.x as f32, n1_v.y as f32, n1_v.z as f32, 0.0];
        let n_c: [f32; 4] = [n2_v.x as f32, n2_v.y as f32, n2_v.z as f32, 0.0];
        let n_b: [f32; 4] = [dir.x as f32, dir.y as f32, dir.z as f32, 0.0];

        {
            let mv0 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        (buffer, indxes, index)
    }
    fn generate_arrow_b_y(
        id: i32,
        dir: Vector3<f64>,
        scale: f64,
        position: Point3<f64>,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let thik = DIM_X_THICKNESS as f32;
        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let fwd_dir = P_FORWARD;
        let up_dir = dir.cross(fwd_dir);

        let cp = position;
        let tmp_p = cp + 3.0 * scale * dir;

        let r_arrow_up = tmp_p + 0.8038 * scale * up_dir;
        let r_arrow_down = tmp_p - 0.8038 * scale * up_dir;

        let n1: [f32; 4] = [1.0, 0.0, 0.0, 0.0];
        let n2: [f32; 4] = [-1.0, 0.0, 0.0, 0.0];

        let n1_v = r_arrow_up.sub(cp).normalize().cross(fwd_dir);
        let n2_v = fwd_dir.cross(r_arrow_down.sub(cp).normalize());
        let n_a: [f32; 4] = [n1_v.x as f32, n1_v.y as f32, n1_v.z as f32, 0.0];
        let n_c: [f32; 4] = [n2_v.x as f32, n2_v.y as f32, n2_v.z as f32, 0.0];
        let n_b: [f32; 4] = [dir.x as f32, dir.y as f32, dir.z as f32, 0.0];

        {
            let mv0 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, cp.y as f32, cp.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [-thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [-thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [thik, r_arrow_up.y as f32, r_arrow_up.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [thik, r_arrow_down.y as f32, r_arrow_down.z as f32, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        (buffer, indxes, index)
    }
}
unsafe impl Send for DimZ {}
unsafe impl Sync for DimZ {}

#[derive(Unique)]
pub struct DimB {
    pub is_dirty: bool,
    pub is_active: bool,
    pub pipe_radius: f64,
    pub pos_y: f64,
    pub angle: f64,
    pub scale: f64,
    pub v_buffer_x: Buffer,
    pub i_buffer_x: Buffer,
    pub v_txt_buffer: Buffer,
    pub diffuse_texture: Texture,
    pub diffuse_texture_view: TextureView,
    pub diffuse_sampler: Sampler,
}
impl DimB {
    pub fn new(device: &Device) -> DimB {
        let diffuse_texture: Texture = device.create_texture(&wgpu::TextureDescriptor {
            // All textures are stored as 3D, we represent our 2D texture
            // by setting depth to 1.
            size: wgpu::Extent3d {
                width: TXT_SIZE_W,
                height: TXT_SIZE_H,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1, // We'll talk about this a little later
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            // Most images are stored using sRGB, so we need to reflect that here.
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            // TEXTURE_BINDING tells wgpu that we want to use this texture in shaders
            // COPY_DST means that we want to copy data to this texture
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: Some("diffuse_texture"),
            // This is the same as with the SurfaceConfig. It
            // specifies what texture formats can be used to
            // create TextureViews for this texture. The base
            // texture format (Rgba8UnormSrgb in this case) is
            // always supported. Note that using a different
            // texture format is not supported on the WebGL2
            // backend.
            view_formats: &[],
        });
        let diffuse_texture_view: TextureView =
            diffuse_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let diffuse_sampler: Sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });
        let v_txt_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("DimX TXT Buffer"),
            size: (TXT_SIZE_W * TXT_SIZE_H * mem::size_of::<f32>() as u32) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let i_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Arrow1 Mesh I Buffer"),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<i32>()) as BufferAddress,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let v_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(format!("Arrow1 Mesh V Buffer {DIM_X_ID}").as_str()),
            size: (BUFFER_SIZE_LIMIT * mem::size_of::<MeshVertex>()) as BufferAddress,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            is_dirty: false,
            is_active: false,
            pipe_radius: 0.0,
            pos_y: 0.0,
            angle: 0.0,
            scale: 1.0,
            v_buffer_x: v_buffer,
            i_buffer_x: i_buffer,
            v_txt_buffer: v_txt_buffer,
            diffuse_texture: diffuse_texture,
            diffuse_texture_view: diffuse_texture_view,
            diffuse_sampler: diffuse_sampler,
        }
    }
    pub fn update(&mut self, device: &Device, queue: &Queue, v_up_orign: &Vector3<f64>) {
        if (self.is_dirty) {
            let mut i: Vec<u32> = vec![];
            let mut v: Vec<MeshVertex> = vec![];
            let mut index: u32 = 0;

            let (rl1, last_index) = DimB::generate_line_b_x(
                DIM_B_ID,
                0.0,
                self.pos_y,
                self.pipe_radius,
                v_up_orign,
                index,
            );
            index = last_index;
            i.extend_from_slice(rl1.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl1.step_vertex_buffer.buffer.as_slice());

            let (rl2, last_index) = DimB::generate_line_b_x(
                DIM_B_ID,
                self.angle,
                self.pos_y,
                self.pipe_radius,
                v_up_orign,
                index,
            );
            index = last_index;
            
            i.extend_from_slice(rl2.step_vertex_buffer.indxes.as_slice());
            v.extend_from_slice(rl2.step_vertex_buffer.buffer.as_slice());

            let (arcv, arci, last_index) = DimB::generate_x_arc(
                DIM_B_ID,
                self.angle,
                self.pos_y,
                self.scale,
                self.pipe_radius,
                v_up_orign,
                index,
            );

            index = last_index;
            i.extend_from_slice(arci.as_slice());
            v.extend_from_slice(arcv.as_slice());

            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&ZEROS_I));
            queue.write_buffer(&self.i_buffer_x, 0, bytemuck::cast_slice(&i));
            queue.write_buffer(&self.v_buffer_x, 0, bytemuck::cast_slice(&v));
            //TXT LAYOUT
            {
                let l = calc_ref_len_offset(self.pipe_radius) * v_up_orign.z;
                let len = l as f32 - 40.0;
                //let len = calc_ref_len_offset(self.pipe_radius) as f32 - 40.0;
                let offsetx: f32 = -20.0;
                let offsetx1: f32 = -20.0 - TXT_SIZE_W as f32;
                let mut v_txt: Vec<MeshVertex> = vec![];
                let ma0 = MeshVertex {
                    position: [offsetx, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma1 = MeshVertex {
                    position: [offsetx, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 0.0],
                };
                let ma2 = MeshVertex {
                    position: [offsetx1, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma3 = MeshVertex {
                    position: [offsetx, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [0.0, 1.0],
                };
                let ma4 = MeshVertex {
                    position: [offsetx1, -(TXT_SIZE_H as f32 + len), 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 0.0],
                };
                let ma5 = MeshVertex {
                    position: [offsetx1, -len, 0.0, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: DIM_X_ID,
                    tex_coords: [1.0, 1.0],
                };
                v_txt.push(ma0);
                v_txt.push(ma1);
                v_txt.push(ma2);
                v_txt.push(ma3);
                v_txt.push(ma4);
                v_txt.push(ma5);
                self.update_glyphs(queue);
                queue.write_buffer(&self.v_txt_buffer, 0, bytemuck::cast_slice(&v_txt));
            }

            self.is_dirty = false;
        }
    }
    fn update_glyphs(&mut self, queue: &Queue) {
        let diffuse_rgba = update_txt(self.angle, self.pipe_radius);
        queue.write_texture(
            // Tells wgpu where to copy the pixel data
            wgpu::ImageCopyTexture {
                texture: &self.diffuse_texture,
                mip_level: 0,
                origin: wgpu::Origin3d {
                    x: 0,
                    y: TXT_SIZE_H - diffuse_rgba.dimensions().1,
                    z: 0,
                },
                aspect: wgpu::TextureAspect::All,
            },
            // The actual pixel data
            &diffuse_rgba,
            // The layout of the texture
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * diffuse_rgba.dimensions().0),
                rows_per_image: Some(diffuse_rgba.dimensions().1),
            },
            wgpu::Extent3d {
                width: diffuse_rgba.dimensions().0,
                height: diffuse_rgba.dimensions().1,
                depth_or_array_layers: 1,
            },
        );
    }
    pub fn set_pipe_radius(&mut self, pipe_radius: f64) {
        self.pipe_radius = pipe_radius;
        self.is_dirty = true;
    }
    pub fn set_y(&mut self, y: f64) {
        self.pos_y = y;
        self.is_dirty = true;
    }
    pub fn set_angle(&mut self, angle: f64) {
        self.angle = angle;
        self.is_dirty = true;
    }
    pub fn set_scale(&mut self, scale: f64) {
        self.scale = scale;
        self.is_dirty = true;
    }
    fn generate_line_b_x(
        id: i32,
        r_deg: f64,
        y: f64,
        pipe_radius: f64,
        v_up_orign: &Vector3<f64>,
        last_index: u32,
    ) -> (MainCylinder, u32) {
        let r = Rad::from(Deg(r_deg));
        let cp: Point3<f64> = Point3::new(0.0, y * v_up_orign.z, 0.0);
        let len = (calc_ref_len(pipe_radius) + y) * v_up_orign.z;

        let rotation: Basis3<f64> = Rotation3::from_axis_angle(P_UP.mul(v_up_orign.z), r);
        let dir_y = rotation.rotate_vector(P_RIGHT_REVERSE);
        let pe: Point3<f64> = cp + dir_y.mul(len);

        let ca = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: cp,
            dir: dir_y,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let cb = MainCircle {
            id: random(),
            radius: DIM_REF_L_RADIUS,
            loc: pe,
            dir: dir_y,
            radius_dir: P_UP,
            r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
        };
        let mut mc = MainCylinder {
            id: id as u64,
            ca: ca,
            cb: cb,
            h: abs(len),
            r: DIM_REF_L_RADIUS,
            r_gr_id: 0,
            ca_tor: 0,
            cb_tor: 0,
            step_vertex_buffer: StepVertexBuffer::default(),
            bbx: BoundingBox::default(),
        };
        let last_index = mc.triangulate_with_start_index_no_cap(last_index);
        (mc, last_index)
    }
    fn generate_arrow_a_x(
        id: i32,
        dir: Vector3<f64>,
        scale: f64,
        position: Point3<f64>,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let thik = DIM_X_THICKNESS as f32;

        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];
        let fwd_dir = P_UP;
        let up_dir = dir.cross(fwd_dir);

        let cp = position;
        let tmp_p = cp + 3.0 * scale * dir;

        let r_arrow_up = tmp_p + 0.8038 * scale * up_dir;
        let r_arrow_down = tmp_p - 0.8038 * scale * up_dir;

        let n1: [f32; 4] = [P_UP.x as f32, P_UP.y as f32, P_UP.z as f32, 0.0];
        let n2: [f32; 4] = [
            P_UP_REVERSE.x as f32,
            P_UP_REVERSE.y as f32,
            P_UP_REVERSE.z as f32,
            0.0,
        ];

        let n1_v = r_arrow_up.sub(cp).normalize().cross(fwd_dir);
        let n2_v = fwd_dir.cross(r_arrow_down.sub(cp).normalize());
        let n_a: [f32; 4] = [n1_v.x as f32, n1_v.y as f32, n1_v.z as f32, 0.0];
        let n_c: [f32; 4] = [n2_v.x as f32, n2_v.y as f32, n2_v.z as f32, 0.0];
        let n_b: [f32; 4] = [dir.x as f32, dir.y as f32, dir.z as f32, 0.0];
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n1,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n2,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_a,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, -thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [cp.x as f32, cp.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_c,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        {
            let mv0 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv0);
            indxes.push(index);
            index = index + 1;

            let mv1 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv1);
            indxes.push(index);
            index = index + 1;

            let mv2 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv2);
            indxes.push(index);
            index = index + 1;

            let mv3 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, -thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv3);
            indxes.push(index);
            index = index + 1;

            let mv4 = MeshVertex {
                position: [r_arrow_up.x as f32, r_arrow_up.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv4);
            indxes.push(index);
            index = index + 1;

            let mv5 = MeshVertex {
                position: [r_arrow_down.x as f32, r_arrow_down.y as f32, thik, 1.0],
                normal: n_b,
                id,
                tex_coords: [0.0, 0.0],
            };
            buffer.push(mv5);
            indxes.push(index);
            index = index + 1;
        }
        (buffer, indxes, index)
    }
    fn generate_x_arc(
        id: i32,
        r_deg: f64,
        y: f64,
        arrow_scale: f64,
        pipe_radius: f64,
        v_up_orign: &Vector3<f64>,
        last_index: u32,
    ) -> (Vec<MeshVertex>, Vec<u32>, u32) {
        let mut index: u32 = last_index;
        let mut indxes: Vec<u32> = vec![];
        let mut buffer: Vec<MeshVertex> = vec![];

        let bend_angle = Rad::from(Deg(r_deg));
        let len = calc_ref_len_offset(pipe_radius) + y;
        let cp: Point3<f64> = Point3::new(0.0, y * v_up_orign.z, 0.0);
        let mut ref_line_a_dir: Vector3<f64> = P_RIGHT_REVERSE.mul(v_up_orign.z);
        let mut pa: Point3<f64> = cp + ref_line_a_dir.mul(len);
        let mut a_dir: Vector3<f64> = P_FORWARD;

        let rot_axe: Vector3<f64> = P_UP.mul(v_up_orign.z);
        let step = Rad::from(Deg(ROTATION_STEP_ANGLE));
        let deg_step = abs((bend_angle.0 / step.0) as i64);
        let mut b_dir: Vector3<f64> = a_dir;
        for i in 0..deg_step {
            let rotation: Basis3<f64> = Rotation3::from_axis_angle(rot_axe, step);
            let ref_line_b_dir = rotation.rotate_vector(ref_line_a_dir);
            let pb: Point3<f64> = cp + ref_line_b_dir.mul(len);
            b_dir = ref_line_b_dir.cross(P_UP_REVERSE);

            let ca = MainCircle {
                id: random(),
                radius: DIM_REF_L_RADIUS,
                loc: pa,
                dir: a_dir,
                radius_dir: P_UP,
                r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
            };
            let cb = MainCircle {
                id: random(),
                radius: DIM_REF_L_RADIUS,
                loc: pb,
                dir: b_dir,
                radius_dir: P_UP,
                r_gr_id: (round_by_dec(DIM_REF_L_RADIUS, 5) * DIVIDER) as u64,
            };

            let arr_a: Vec<Point3<f64>> = ca.gen_points();
            let arr_b: Vec<Point3<f64>> = cb.gen_points();

            for i in 0..arr_a.iter().len() - 1 {
                let p0 = arr_a[i];
                let p1 = arr_a[i + 1];
                let p2 = arr_b[i];
                let p3 = arr_b[i + 1];

                let plane = Plane::new(p0.clone(), p1.clone(), p3.clone());
                let n = plane.normal().normalize();
                let mut n_arr = [n.x as f32, n.y as f32, n.z as f32, 0.0];
                let r_dir = p0.sub(ca.loc);
                let is_coplanar = n.dot(r_dir);
                if (is_coplanar < 0.0) {
                    //let n_rev = plane.normal().normalize().mul(-1.0);
                    n_arr = [-n_arr[0], -n_arr[1], -n_arr[2], 0.0];
                }

                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p1.x as f32, p1.y as f32, p1.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
                {
                    let mv0 = MeshVertex {
                        position: [p0.x as f32, p0.y as f32, p0.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv0);
                    indxes.push(index);
                    index = index + 1;

                    let mv1 = MeshVertex {
                        position: [p3.x as f32, p3.y as f32, p3.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv1);
                    indxes.push(index);
                    index = index + 1;

                    let mv2 = MeshVertex {
                        position: [p2.x as f32, p2.y as f32, p2.z as f32, 1.0],
                        normal: n_arr,
                        id: id,
                        tex_coords: [0.0, 0.0],
                    };
                    buffer.push(mv2);
                    indxes.push(index);
                    index = index + 1;
                }
            }
            ref_line_a_dir = ref_line_b_dir;
            pa = pb;
            a_dir = b_dir;
        }

        let parrow1 = cp + P_RIGHT_REVERSE.mul(len * v_up_orign.z);
        let (buffer_a1, indxes_a1, last_index_a1) =
            DimB::generate_arrow_a_x(id, P_FORWARD_REVERSE, arrow_scale, parrow1, index);
        index = last_index_a1;
        indxes.extend_from_slice(indxes_a1.as_slice());
        buffer.extend_from_slice(buffer_a1.as_slice());

        let (buffer_a2, indxes_a2, last_index_a2) =
            DimB::generate_arrow_a_x(id, b_dir.mul(v_up_orign.z), arrow_scale, pa, index);
        index = last_index_a2;

        indxes.extend_from_slice(indxes_a2.as_slice());
        buffer.extend_from_slice(buffer_a2.as_slice());

        (buffer, indxes, index)
    }
}
unsafe impl Send for DimB {}
unsafe impl Sync for DimB {}

fn calc_ref_len(pipe_radius: f64) -> f64 {
    pipe_radius * REF_LINE_L_FACTOR
}

fn calc_ref_len_offset(pipe_radius: f64) -> f64 {
    pipe_radius * REF_LINE_L_FACTOR - pipe_radius * REF_LINE_OFFSET_FACTOR
}

fn update_txt(txt_value: f64, pipe_radius: f64) -> RgbaImage {
    let font = Font::try_from_bytes(FONT_ROBOTO_REGULAR).expect("Error constructing Font");
    let font_h = pipe_radius * TXT_H_RADIUS_FACTOR;
    let scale = Scale::uniform(font_h as f32);
    let text = (txt_value as i32).to_string();
    let colour = (150 as u8, 0 as u8, 0 as u8);
    let v_metrics = font.v_metrics(scale);
    let glyphs: Vec<_> = font
        .layout(text.as_str(), scale, point(0.0, 0.0 + v_metrics.ascent))
        .collect();
    let glyphs_height = (v_metrics.ascent - v_metrics.descent).ceil() as u32; //512 - 40 as u32; //
    let glyphs_width = {
        let min_x = glyphs
            .first()
            .map(|g| g.pixel_bounding_box().unwrap().min.x)
            .unwrap();
        let max_x = glyphs
            .last()
            .map(|g| g.pixel_bounding_box().unwrap().max.x)
            .unwrap();
        let w = (max_x - min_x) as u32;
        if (w % 4 == 0) {
            w
        } else {
            (w / 4) * 4 + 4
        }
    };
    let mut diffuse_rgba: RgbaImage =
        DynamicImage::new_rgba8(glyphs_width + 120, glyphs_height + 40).to_rgba8();
    for glyph in glyphs {
        if let Some(bounding_box) = glyph.pixel_bounding_box() {
            glyph.draw(|x, y, v| {
                diffuse_rgba.put_pixel(
                    x + bounding_box.min.x as u32,
                    y + bounding_box.min.y as u32,
                    Rgba([colour.0, colour.1, colour.2, (v * 255.0) as u8]),
                )
            });
        }
    }
    diffuse_rgba
}
