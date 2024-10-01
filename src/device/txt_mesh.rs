use std::f64::consts::PI;
use cgmath::{Deg, Matrix4, Rad, SquareMatrix, Vector3, Vector4};
use log::warn;
use wgpu::{Buffer, Device};
use crate::device::{MeshVertex, StepVertexBuffer};
use meshtext::{Glyph, IndexedMeshText, MeshGenerator};

use wgpu::util::DeviceExt;
use crate::device::dorn::SEMI_LENGTH;
use crate::device::gstate::{FORWARD_DIR32, RIGHT_DIR32, UP_DIR32};
use crate::device::mesh_pipeline::{TXT_A_ID, TXT_B_ID, TXT_C_ID};

pub const FONT_SIZE: f32 = 150.0;
pub const FONT_OFFSET: f32 = 0.4;
pub const CHARACTERS: [char; 11] = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '-'];

pub struct TxtMesh {
    pub is_dirty: bool,

    digits_atlas_a_0: Vec<StepVertexBuffer>,
    digits_atlas_a_1: Vec<StepVertexBuffer>,
    digits_atlas_a_2: Vec<StepVertexBuffer>,
    digits_atlas_a_3: Vec<StepVertexBuffer>,
    pub txt_vertex_buffer_a: Vec<StepVertexBuffer>,
    pub v_txt_buffer_a: Vec<Buffer>,
    pub i_txt_buffer_a: Vec<Buffer>,
    pub txt_a_transformation: Matrix4<f32>,

    digits_atlas_b_0: Vec<StepVertexBuffer>,
    digits_atlas_b_1: Vec<StepVertexBuffer>,
    digits_atlas_b_2: Vec<StepVertexBuffer>,
    digits_atlas_b_3: Vec<StepVertexBuffer>,
    pub txt_vertex_buffer_b: Vec<StepVertexBuffer>,
    pub v_txt_buffer_b: Vec<Buffer>,
    pub i_txt_buffer_b: Vec<Buffer>,
    pub txt_b_transformation: Matrix4<f32>,

    digits_atlas_c_0: Vec<StepVertexBuffer>,
    digits_atlas_c_1: Vec<StepVertexBuffer>,
    digits_atlas_c_2: Vec<StepVertexBuffer>,
    digits_atlas_c_3: Vec<StepVertexBuffer>,
    pub txt_vertex_buffer_c: Vec<StepVertexBuffer>,
    pub v_txt_buffer_c: Vec<Buffer>,
    pub i_txt_buffer_c: Vec<Buffer>,
    pub txt_c_transformation: Matrix4<f32>,

}
impl TxtMesh {
    pub fn new() -> TxtMesh {
        let font_data = include_bytes!("../fonts/FiraMono-Regular.ttf");
        let mut generator = MeshGenerator::new(font_data);
        let mut digits_atlas_a_0: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_a_1: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_a_2: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_a_3: Vec<StepVertexBuffer> = vec![];

        let mut digits_atlas_b_0: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_b_1: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_b_2: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_b_3: Vec<StepVertexBuffer> = vec![];

        let mut digits_atlas_c_0: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_c_1: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_c_2: Vec<StepVertexBuffer> = vec![];
        let mut digits_atlas_c_3: Vec<StepVertexBuffer> = vec![];


        let id_a = TXT_A_ID as i32;
        let id_b = TXT_B_ID as i32;
        let id_c = TXT_C_ID as i32;

        let font_offset: f32 = FONT_SIZE * 0.4;
        CHARACTERS.iter().for_each(|character| {
            let mut new_idxses: Vec<u32> = vec![];
            let mut new_idx: u32 = 0;
            let mut mvs_0_a: Vec<MeshVertex> = vec![];
            let mut mvs_1_a: Vec<MeshVertex> = vec![];
            let mut mvs_2_a: Vec<MeshVertex> = vec![];
            let mut mvs_3_a: Vec<MeshVertex> = vec![];

            let mut mvs_0_b: Vec<MeshVertex> = vec![];
            let mut mvs_1_b: Vec<MeshVertex> = vec![];
            let mut mvs_2_b: Vec<MeshVertex> = vec![];
            let mut mvs_3_b: Vec<MeshVertex> = vec![];

            let mut mvs_0_c: Vec<MeshVertex> = vec![];
            let mut mvs_1_c: Vec<MeshVertex> = vec![];
            let mut mvs_2_c: Vec<MeshVertex> = vec![];
            let mut mvs_3_c: Vec<MeshVertex> = vec![];

            let result: IndexedMeshText = generator.generate_glyph(character.clone(), true, None).unwrap();
            result.indices.iter().for_each(|inds| {
                let y = result.vertices[(inds * 3 + 1) as usize] * FONT_SIZE;
                let z = result.vertices[(inds * 3 + 2) as usize] * FONT_SIZE;

                let x_0 = result.vertices[(inds * 3) as usize] * FONT_SIZE;
                let x_1 = result.vertices[(inds * 3) as usize] * FONT_SIZE + font_offset;
                let x_2 = result.vertices[(inds * 3) as usize] * FONT_SIZE + font_offset * 2.0;
                let x_3 = result.vertices[(inds * 3) as usize] * FONT_SIZE + font_offset * 3.0;

                new_idxses.push(new_idx);
                new_idx = new_idx + 1;

                let mv_0_a = MeshVertex {
                    position: [x_0, y, z, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: id_a,
                };
                let mv_1_a = MeshVertex {
                    position: [x_1, y, z, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: id_a,
                };
                let mv_2_a = MeshVertex {
                    position: [x_2, y, z, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: id_a,
                };
                let mv_3_a = MeshVertex {
                    position: [x_3, y, z, 1.0],
                    normal: [0.0, 0.0, 1.0, 0.0],
                    id: id_a,
                };

                let mut mv_0_b = mv_0_a.clone();
                mv_0_b.id = id_b;
                let mut mv_1_b = mv_1_a.clone();
                mv_1_b.id = id_b;
                let mut mv_2_b = mv_2_a.clone();
                mv_2_b.id = id_b;
                let mut mv_3_b = mv_3_a.clone();
                mv_3_b.id = id_b;

                let mut mv_0_c = mv_0_a.clone();
                mv_0_c.id = id_c;
                let mut mv_1_c = mv_1_a.clone();
                mv_1_c.id = id_c;
                let mut mv_2_c = mv_2_a.clone();
                mv_2_c.id = id_c;
                let mut mv_3_c = mv_3_a.clone();
                mv_3_c.id = id_c;

                mvs_0_a.push(mv_0_a);
                mvs_1_a.push(mv_1_a);
                mvs_2_a.push(mv_2_a);
                mvs_3_a.push(mv_3_a);

                mvs_0_b.push(mv_0_b);
                mvs_1_b.push(mv_1_b);
                mvs_2_b.push(mv_2_b);
                mvs_3_b.push(mv_3_b);

                mvs_0_c.push(mv_0_c);
                mvs_1_c.push(mv_1_c);
                mvs_2_c.push(mv_2_c);
                mvs_3_c.push(mv_3_c);
            });
            let sv_0_a = StepVertexBuffer {
                buffer: mvs_0_a,
                indxes: new_idxses.clone(),
            };
            let sv_1_a = StepVertexBuffer {
                buffer: mvs_1_a,
                indxes: new_idxses.clone(),
            };
            let sv_2_a = StepVertexBuffer {
                buffer: mvs_2_a,
                indxes: new_idxses.clone(),
            };
            let sv_3_a = StepVertexBuffer {
                buffer: mvs_3_a,
                indxes: new_idxses.clone(),
            };

            let sv_0_b = StepVertexBuffer {
                buffer: mvs_0_b,
                indxes: new_idxses.clone(),
            };
            let sv_1_b = StepVertexBuffer {
                buffer: mvs_1_b,
                indxes: new_idxses.clone(),
            };
            let sv_2_b = StepVertexBuffer {
                buffer: mvs_2_b,
                indxes: new_idxses.clone(),
            };
            let sv_3_b = StepVertexBuffer {
                buffer: mvs_3_b,
                indxes: new_idxses.clone(),
            };

            let sv_0_c = StepVertexBuffer {
                buffer: mvs_0_c,
                indxes: new_idxses.clone(),
            };
            let sv_1_c = StepVertexBuffer {
                buffer: mvs_1_c,
                indxes: new_idxses.clone(),
            };
            let sv_2_c = StepVertexBuffer {
                buffer: mvs_2_c,
                indxes: new_idxses.clone(),
            };
            let sv_3_c = StepVertexBuffer {
                buffer: mvs_3_c,
                indxes: new_idxses.clone(),
            };

            digits_atlas_a_0.push(sv_0_a);
            digits_atlas_a_1.push(sv_1_a);
            digits_atlas_a_2.push(sv_2_a);
            digits_atlas_a_3.push(sv_3_a);

            digits_atlas_b_0.push(sv_0_b);
            digits_atlas_b_1.push(sv_1_b);
            digits_atlas_b_2.push(sv_2_b);
            digits_atlas_b_3.push(sv_3_b);

            digits_atlas_c_0.push(sv_0_c);
            digits_atlas_c_1.push(sv_1_c);
            digits_atlas_c_2.push(sv_2_c);
            digits_atlas_c_3.push(sv_3_c);
        });

        let mut txt_vertex_buffer_a: Vec<StepVertexBuffer> = vec![];
        /*      txt_vertex_buffer_a.push(digits_atlas_a_0[5].clone());
              txt_vertex_buffer_a.push(digits_atlas_a_1[6].clone());
              txt_vertex_buffer_a.push(digits_atlas_a_2[1].clone());
              txt_vertex_buffer_a.push(digits_atlas_a_3[10].clone());*/

        let mut txt_vertex_buffer_b: Vec<StepVertexBuffer> = vec![];
        txt_vertex_buffer_b.push(digits_atlas_b_0[2].clone());
        txt_vertex_buffer_b.push(digits_atlas_b_1[0].clone());
        txt_vertex_buffer_b.push(digits_atlas_b_2[4].clone());
        txt_vertex_buffer_b.push(digits_atlas_b_3[7].clone());

        let mut txt_vertex_buffer_c: Vec<StepVertexBuffer> = vec![];
        /*        txt_vertex_buffer_c.push(digits_atlas_c_0[7].clone());
                txt_vertex_buffer_c.push(digits_atlas_c_1[7].clone());
                txt_vertex_buffer_c.push(digits_atlas_c_2[8].clone());
                txt_vertex_buffer_c.push(digits_atlas_c_3[1].clone());*/
        Self {
            is_dirty: true,
            digits_atlas_a_0: digits_atlas_a_0,
            digits_atlas_a_1: digits_atlas_a_1,
            digits_atlas_a_2: digits_atlas_a_2,
            digits_atlas_a_3: digits_atlas_a_3,
            txt_vertex_buffer_a: txt_vertex_buffer_a,
            v_txt_buffer_a: vec![],
            i_txt_buffer_a: vec![],
            txt_a_transformation: Matrix4::identity(),

            digits_atlas_b_0: digits_atlas_b_0,
            digits_atlas_b_1: digits_atlas_b_1,
            digits_atlas_b_2: digits_atlas_b_2,
            digits_atlas_b_3: digits_atlas_b_3,
            txt_vertex_buffer_b: txt_vertex_buffer_b,
            v_txt_buffer_b: vec![],
            i_txt_buffer_b: vec![],
            txt_b_transformation: Matrix4::identity(),

            digits_atlas_c_0: digits_atlas_c_0,
            digits_atlas_c_1: digits_atlas_c_1,
            digits_atlas_c_2: digits_atlas_c_2,
            digits_atlas_c_3: digits_atlas_c_3,
            txt_vertex_buffer_c: txt_vertex_buffer_c,
            v_txt_buffer_c: vec![],
            i_txt_buffer_c: vec![],
            txt_c_transformation: Matrix4::identity(),
        }
    }

    pub fn setup(&mut self) {
        self.setup_a_pos(0.0);
        self.setup_b_pos(0.0, 0.0);
        self.setup_c_pos(0.0);
    }

    pub fn setup_a_pos(&mut self, angle_deg: f64) {
        let glyph_count: usize = (angle_deg as i32).to_string().len();
        let middle_point = -(glyph_count as f32 / 4.0 * FONT_SIZE + (glyph_count as f32 - 1.0) * FONT_OFFSET);
        let tr_middle_point = Matrix4::from_translation(Vector3::new(middle_point, 0.0, 0.0));
        let rad = angle_deg.to_radians();
        let rot1: Matrix4<f32> = Matrix4::from_axis_angle(RIGHT_DIR32, Rad::from(Deg(90.0)));
        let rot2: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(Deg(90.0)));
        let v: Vector3<f32> = Vector3::new(0.0, 0.0, SEMI_LENGTH as f32);
        let rot3: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad((rad / 2.0) as f32));
        let tr = Matrix4::from_translation(v);
        self.txt_a_transformation = rot3 * tr * rot2 * rot1 * tr_middle_point;
        self.set_digit_a(angle_deg as i32);
    }

    pub fn setup_b_pos(&mut self, angle_deg: f64, dorn_radius: f64) {
        let dorn_offset: Matrix4<f32> = Matrix4::from_translation(Vector3::new(0.0, dorn_radius as f32, 0.0));
        warn!("BEND ANGLE {:?}",angle_deg);
        let glyph_count: usize = (angle_deg as i32).to_string().len();
        let middle_point = -(glyph_count as f32 / 4.0 * FONT_SIZE + (glyph_count as f32 - 1.0) * FONT_OFFSET);
        let tr_middle_point = Matrix4::from_translation(Vector3::new(middle_point, 0.0, 0.0));
        let rad = angle_deg.to_radians();
        let rot1: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad::from(Deg(180.0)));
        //let rot2: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(Deg(90.0)));
        let v: Vector3<f32> = Vector3::new(0.0, -SEMI_LENGTH as f32, 0.0);
        let rot3: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad((rad / 2.0) as f32));
        let tr = Matrix4::from_translation(v);
        self.txt_b_transformation = rot3 * tr * dorn_offset * rot1 * tr_middle_point; //Matrix4::identity();
        self.set_digit_b(angle_deg as i32);
    }

    pub fn setup_c_pos(&mut self, angle_deg: f64) {
        let dorn_offset: Matrix4<f32> = Matrix4::from_translation(Vector3::new(angle_deg as f32 / 2.0, 0.0, 0.0));
        warn!("BEND ANGLE {:?}",angle_deg);
        let glyph_count: usize = (angle_deg as i32).to_string().len();
        let middle_point = -(glyph_count as f32 / 4.0 * FONT_SIZE + (glyph_count as f32 - 1.0) * FONT_OFFSET);
        let tr_middle_point = Matrix4::from_translation(Vector3::new(middle_point, 0.0, 0.0));
        let rad = angle_deg.to_radians();
        let rot1: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad::from(Deg(180.0)));
        //let rot2: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(Deg(90.0)));
        let v: Vector3<f32> = Vector3::new(0.0, -SEMI_LENGTH as f32, 0.0);
        let rot3: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad((rad / 2.0) as f32));
        let tr = Matrix4::from_translation(v);
        self.txt_c_transformation = tr * dorn_offset * rot1 * tr_middle_point; //Matrix4::identity();
        self.set_digit_c(angle_deg as i32);
    }
    pub fn set_digit_a(&mut self, digit: i32) {
        let mut txt_vertex_buffer_a: Vec<StepVertexBuffer> = vec![];
        if (digit != 0) {
            let mut counter = 0;
            digit.to_string().chars().for_each(|character| {
                let char_indx: usize = {
                    match character {
                        '0' => { 0 }
                        '1' => { 1 }
                        '2' => { 2 }
                        '3' => { 3 }
                        '4' => { 4 }
                        '5' => { 5 }
                        '6' => { 6 }
                        '7' => { 7 }
                        '8' => { 8 }
                        '9' => { 9 }
                        '-' => { 10 }
                        _ => { 100 }
                    }
                };

                if (char_indx != 100) {
                    match counter {
                        0 => { txt_vertex_buffer_a.push(self.digits_atlas_a_0[char_indx].clone()); }
                        1 => { txt_vertex_buffer_a.push(self.digits_atlas_a_1[char_indx].clone()); }
                        2 => { txt_vertex_buffer_a.push(self.digits_atlas_a_2[char_indx].clone()); }
                        3 => { txt_vertex_buffer_a.push(self.digits_atlas_a_3[char_indx].clone()); }
                        _ => {}
                    }
                }
                counter = counter + 1;
            });
        }
        self.txt_vertex_buffer_a = txt_vertex_buffer_a;
        self.is_dirty = true;
    }
    pub fn set_digit_b(&mut self, digit: i32) {
        let mut txt_vertex_buffer: Vec<StepVertexBuffer> = vec![];
        if (digit != 0) {
            let mut counter = 0;
            digit.to_string().chars().for_each(|character| {
                let char_indx: usize = {
                    match character {
                        '0' => { 0 }
                        '1' => { 1 }
                        '2' => { 2 }
                        '3' => { 3 }
                        '4' => { 4 }
                        '5' => { 5 }
                        '6' => { 6 }
                        '7' => { 7 }
                        '8' => { 8 }
                        '9' => { 9 }
                        '-' => { 10 }
                        _ => { 100 }
                    }
                };

                if (char_indx != 100) {
                    match counter {
                        0 => { txt_vertex_buffer.push(self.digits_atlas_b_0[char_indx].clone()); }
                        1 => { txt_vertex_buffer.push(self.digits_atlas_b_1[char_indx].clone()); }
                        2 => { txt_vertex_buffer.push(self.digits_atlas_b_2[char_indx].clone()); }
                        3 => { txt_vertex_buffer.push(self.digits_atlas_b_3[char_indx].clone()); }
                        _ => {}
                    }
                }
                counter = counter + 1;
            });
        }
        self.txt_vertex_buffer_b = txt_vertex_buffer;
        self.is_dirty = true;
    }
    pub fn set_digit_c(&mut self, digit: i32) {
        let mut txt_vertex_buffer: Vec<StepVertexBuffer> = vec![];
        if (digit != 0) {
            let mut counter = 0;
            digit.to_string().chars().for_each(|character| {
                let char_indx: usize = {
                    match character {
                        '0' => { 0 }
                        '1' => { 1 }
                        '2' => { 2 }
                        '3' => { 3 }
                        '4' => { 4 }
                        '5' => { 5 }
                        '6' => { 6 }
                        '7' => { 7 }
                        '8' => { 8 }
                        '9' => { 9 }
                        '-' => { 10 }
                        _ => { 100 }
                    }
                };

                if (char_indx != 100) {
                    match counter {
                        0 => { txt_vertex_buffer.push(self.digits_atlas_c_0[char_indx].clone()); }
                        1 => { txt_vertex_buffer.push(self.digits_atlas_c_1[char_indx].clone()); }
                        2 => { txt_vertex_buffer.push(self.digits_atlas_c_2[char_indx].clone()); }
                        3 => { txt_vertex_buffer.push(self.digits_atlas_c_3[char_indx].clone()); }
                        _ => {}
                    }
                }
                counter = counter + 1;
            });
        }

        self.txt_vertex_buffer_c = txt_vertex_buffer;
        self.is_dirty = true;
    }
    pub fn update_vertexes(&mut self, device: &Device) {
        self.i_txt_buffer_a = vec![];
        self.v_txt_buffer_a = vec![];
        let mut index = 0;
        self.txt_vertex_buffer_a.iter().for_each(|item| {
            let i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Index Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.indxes),
                usage: wgpu::BufferUsages::INDEX,
            });
            let v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Vertex Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.buffer),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.i_txt_buffer_a.push(i_buffer);
            self.v_txt_buffer_a.push(v_buffer);
            index = index + 1;
        });

        self.i_txt_buffer_b = vec![];
        self.v_txt_buffer_b = vec![];
        index = 0;
        self.txt_vertex_buffer_b.iter().for_each(|item| {
            let i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Index Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.indxes),
                usage: wgpu::BufferUsages::INDEX,
            });
            let v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Vertex Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.buffer),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.i_txt_buffer_b.push(i_buffer);
            self.v_txt_buffer_b.push(v_buffer);
            index = index + 1;
        });

        self.i_txt_buffer_c = vec![];
        self.v_txt_buffer_c = vec![];
        index = 0;
        self.txt_vertex_buffer_c.iter().for_each(|item| {
            let i_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Index Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.indxes),
                usage: wgpu::BufferUsages::INDEX,
            });
            let v_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("Vertex Txt Buffer {index}").as_str()),
                contents: bytemuck::cast_slice(&item.buffer),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.i_txt_buffer_c.push(i_buffer);
            self.v_txt_buffer_c.push(v_buffer);
            index = index + 1;
        });

        self.is_dirty = false;
    }
    pub fn test_transform_a(&mut self) {
        let rot: Matrix4<f32> = Matrix4::from_axis_angle(UP_DIR32, Rad::from(Deg(90.0)));
        self.txt_a_transformation = rot;
        self.is_dirty = true;
    }
    pub fn test_transform_b(&mut self) {
        let rot: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(Deg(45.0)));
        self.txt_b_transformation = rot;
        self.is_dirty = true;
    }
    pub fn test_transform_c(&mut self) {
        let rot: Matrix4<f32> = Matrix4::from_axis_angle(FORWARD_DIR32, Rad::from(Deg(-45.0)));
        self.txt_c_transformation = rot;
        self.is_dirty = true;
    }
}