use crate::algo::cnc::{all_to_one, all_to_stp, cnc_to_poly, LRACLR};
use crate::algo::{analyze_stp, cnc, BendToro, MainCylinder, P_UP, P_UP_REVERSE};
use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::camera::Camera;
use crate::device::graphics::States::{ChangeDornDir, Dismiss, FullAnimate, LoadLRA, NewBendParams, ReadyToLoad, ReverseLRACLR, SelectFromWeb};
use crate::device::mesh_pipeline::MeshPipeLine;
use crate::device::txt_pipeline::TxtPipeLine;
use crate::device::MeshVertex;
#[cfg(target_arch = "wasm32")]
use crate::remote::in_state::bend_settings;
#[cfg(target_arch = "wasm32")]
use crate::remote::in_state::change_bend_step;
#[cfg(target_arch = "wasm32")]
use crate::remote::in_state::{pipe_bend_ops, InCmd};
use crate::utils::dim::{DimB, DimX, DimZ};
use crate::utils::dorn::Dorn;
use cgmath::num_traits::{abs, signum};
use cgmath::{Deg, Point3, Rad};
use log::warn;
//use once_cell::sync::Lazy;
use shipyard::{EntitiesViewMut, EntityId, Unique, UniqueViewMut, ViewMut, World};
use smaa::{SmaaFrame, SmaaMode, SmaaTarget};
use std::collections::HashMap;
use std::fmt::{Debug, Formatter};
use std::ops::{Mul, Range};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::{iter, mem};
use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Write};
use is_odd::IsOdd;
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::Vector3;
use truck_modeling::Curve;
use truck_stepio::out;
use truck_stepio::out::StepModel;
use truck_topology::Solid;
use web_sys::js_sys::Float32Array;
use web_sys::HtmlCanvasElement;
use web_time::{Instant, SystemTime};
use wgpu::util::DeviceExt;
use wgpu::{
    Adapter, BindGroup, BindingResource, Buffer, BufferAddress, BufferAsyncError, BufferSlice,
    CommandEncoder, Device, Queue, RenderPass, StoreOp, Surface, SurfaceConfiguration, Texture,
    TextureFormat, TextureView, TextureViewDescriptor, COPY_BYTES_PER_ROW_ALIGNMENT,
};
use winit::dpi::{PhysicalPosition, PhysicalSize};
use winit::event::{ElementState, KeyEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::Window;
use crate::utils::materials::{Material, MATERIALS_COUNT};

const METADATA_COUNT: usize = 256;
const STRIGHT_COLOR: u32 = 76;
const BEND_COLOR: u32 = 37;
pub static IS_OFFSCREEN_BUFFER_MAPPED: AtomicBool = AtomicBool::new(false);
pub const OFFSCREEN_TEXEL_SIZE: u32 = 16;
const MESH_BUFFER_LIMIT: usize = 2000000;
const MESH_ZEROS_I: [i32; MESH_BUFFER_LIMIT] = [0; MESH_BUFFER_LIMIT];
const FRAMERATE_SECONDS: f64 = 0.1;
const SELECTION_COLOR: i32 = 1;
const BACKGROUND_COLOR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
const BACKGROUND_COLOR_SELECTION: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
#[derive(Clone)]
pub enum States {
    Dismiss,
    ReadyToLoad((Vec<LRACLR>, bool)),
    FullAnimate,
    ReverseLRACLR,
    ChangeDornDir,
    LoadLRA(Vec<f32>),
    NewBendParams(Vec<f32>),
    SelectFromWeb(i32),
}
pub struct AnimState {
    pub id: i32,
    pub opcode: usize,
    pub value: f64,
    pub stright_len: f64,
    pub lra: LRACLR,
    pub op_counter: i32,
}
impl AnimState {
    pub fn new(
        id: i32,
        opcode: usize,
        value: f64,
        stright_len: f64,
        lra: LRACLR,
        op_counter: i32,
    ) -> Self {
        Self {
            id,
            opcode,
            value,
            stright_len,
            lra,
            op_counter: op_counter,
        }
    }
    pub fn default() -> Self {
        Self {
            id: 0,
            opcode: 5,
            value: 0.0,
            stright_len: 0.0,
            lra: LRACLR::default(),
            op_counter: 0,
        }
    }
}
impl Debug for AnimState {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "id {} opcode {} value {}",
            self.id, self.opcode, self.value
        )
    }
}
pub struct BendParameters {
    pub stright_speed: f64,
    pub rotate_speed: f64,
    pub angle_speed: f64,
}
impl BendParameters {
    pub fn default() -> Self {
        Self {
            stright_speed: 100.0,
            rotate_speed: 10.0,
            angle_speed: 10.0,
        }
    }
    pub fn set_params_from_f32vec(&mut self, vec: &Vec<f32>) {
        match vec.get(0) {
            None => {}
            Some(v) => self.stright_speed = v.clone() as f64,
        }
        match vec.get(1) {
            None => {}
            Some(v) => self.rotate_speed = v.clone() as f64,
        }
        match vec.get(2) {
            None => {}
            Some(v) => self.angle_speed = v.clone() as f64,
        }
    }
    pub fn params_to_f32vec(&self) -> Vec<f32> {
        Vec::from([
            self.stright_speed as f32,
            self.rotate_speed as f32,
            self.angle_speed as f32,
        ])
    }
}

#[derive(Unique)]
pub struct GlobalState {
    pub is_right_mouse_pressed: bool,
    pub state: States,
    pub prev_state: States,
    pub lraclr_arr: Vec<LRACLR>,
    pub lraclr_arr_reversed: Vec<LRACLR>,
    pub idmaps: HashMap<u64, EntityId>,
    pub anim_state: AnimState,
    pub v_up_orign: Vector3,
    pub instant: Instant,
    pub dt: f64,
    pub is_next_frame_ready: bool,
    pub smaa_target: SmaaTarget,
    pub is_reversed: bool,
}
impl GlobalState {
    pub fn check_framerate(&mut self) {
        let dt = self.instant.elapsed().as_millis() as f64 / 1000.0;
        self.dt = dt;
        self.instant = Instant::now();
    }
    pub fn calculate_unbend_bbx(&self) -> BoundingBox<Point3<f64>> {
        let mut tot_x: f64 = 0.0;
        let mut pipe_radius: f64 = 0.0;
        self.lraclr_arr.iter().for_each(|lracl| {
            tot_x = tot_x + lracl.l;
            tot_x = tot_x + lracl.lt;
            pipe_radius = lracl.pipe_radius;
        });
        let half = tot_x / 2.0;
        let mut bbx: BoundingBox<cgmath::Point3<f64>> = BoundingBox::new();
        bbx.push(cgmath::Point3::new(0.0, half, half));
        bbx.push(cgmath::Point3::new(tot_x, -half, -half));
        bbx
    }
    pub fn calculate_total_len(&self) -> f64 {
        let mut tot_x: f64 = 0.0;
        self.lraclr_arr.iter().for_each(|lracl| {
            tot_x = tot_x + lracl.l;
            tot_x = tot_x + lracl.lt;
        });
        tot_x
    }

    pub fn change_state(&mut self, new_state: States) -> States {
        self.prev_state = self.state.clone();
        self.state = new_state;
        self.state.clone()
    }
    pub fn revert_state(&mut self) -> States {
        self.state = self.prev_state.clone();
        self.state.clone()
    }
}
unsafe impl Send for GlobalState {}
unsafe impl Sync for GlobalState {}

#[derive(Unique)]
pub struct GlobalScene {
    pub bend_step: usize,
    pub dorn: Dorn,
    pub dim_x: DimX,
    pub dim_z: DimZ,
    pub dim_b: DimB,
    pub bend_params: BendParameters,
    pub mesh_size: usize,
    pub v_buffer_mesh: Buffer,
    pub i_buffer_mesh: Buffer,
    pub offscreen_width: u32,
    pub offscreen_data: Vec<i32>,
    pub offscreen_buffer: Buffer,
    pub is_offscreen_requested: bool,
    pub mouse_x: f64,
    pub mouse_y: f64,
    pub camera_buffer: Buffer,
    pub material_buffer: Buffer,
    pub materials: Vec<Material>,
    pub light_buffer: Buffer,
    pub metadata: Vec<[i32; 4]>,
    pub metadata_buffer: Buffer,
}
impl GlobalScene {
    pub fn new(device: &Device, queue: &Queue, v_up_orign: &cgmath::Vector3<f64>, w: u32, h: u32) -> Self {
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
        let mut metadata_default: Vec<[i32; 4]> = vec![];
        for i in 0..METADATA_COUNT {
            if (!i.is_odd()) {
                metadata_default.push([STRIGHT_COLOR as i32, 0, 0, 0]);
            } else {
                metadata_default.push([BEND_COLOR as i32, 0, 0, 0]);
            }
        }
        let metadata_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("Vertex Mesh Buffer").as_str()),
            contents: bytemuck::cast_slice(&metadata_default),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let i_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Mesh I Buffer"),
            size: (MESH_BUFFER_LIMIT * mem::size_of::<i32>()) as BufferAddress,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::VERTEX,
            mapped_at_creation: false,
        });
        let v_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(format!("Mesh V Buffer").as_str()),
            size: (MESH_BUFFER_LIMIT * mem::size_of::<MeshVertex>()) as BufferAddress,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let offscreen_width: u32 = GlobalScene::calculate_offset_pad(w as u32);
        let mut offscreen_data: Vec<i32> = Vec::<i32>::with_capacity((offscreen_width * h * OFFSCREEN_TEXEL_SIZE) as usize);
        let offscreen_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("OFSSCREEN_BUFFER"),
            size: offscreen_data.capacity() as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let materials = Material::generate_materials();
        queue.write_buffer(&material_buffer, 0, bytemuck::cast_slice(&materials));
        Self {
            bend_step: 1,
            dorn: Dorn::new(&device, &v_up_orign),
            dim_x: DimX::new(&device),
            dim_z: DimZ::new(&device),
            dim_b: DimB::new(&device),
            bend_params: BendParameters::default(),
            mesh_size: 0,
            v_buffer_mesh: i_buffer,
            i_buffer_mesh: v_buffer,
            offscreen_width: offscreen_width,
            offscreen_data: offscreen_data,
            offscreen_buffer: offscreen_buffer,
            is_offscreen_requested: false,
            mouse_x: 0.0,
            mouse_y: 0.0,
            camera_buffer: camera_buffer,
            material_buffer: material_buffer,
            materials: materials,
            light_buffer: light_buffer,
            metadata: metadata_default,
            metadata_buffer: metadata_buffer,
        }
    }
    pub fn resize(&mut self, device: &Device, w: u32, h: u32) {
        let offscreen_width: u32 = GlobalScene::calculate_offset_pad(w as u32);
        let mut offscreen_data: Vec<i32> = Vec::<i32>::with_capacity((offscreen_width * h * OFFSCREEN_TEXEL_SIZE) as usize);
        let offscreen_buffer: Buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("OFSSCREEN_BUFFER"),
            size: offscreen_data.capacity() as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        self.offscreen_width = offscreen_width;
        self.offscreen_data = offscreen_data;
        self.offscreen_buffer = offscreen_buffer;
        self.is_offscreen_requested = false;

        IS_OFFSCREEN_BUFFER_MAPPED.store(false, Ordering::Relaxed);
    }
    pub fn unselect_all(&mut self, queue: &Queue) {
        self.metadata.iter_mut().for_each(|md| { md[1] = 0; });
        self.select_by_id(queue, -1);

        #[cfg(target_arch = "wasm32")]{
            use crate::remote::in_state::select_by_id;
            select_by_id(-1);
        }
    }
    pub fn select_by_id(&mut self, queue: &Queue, id: i32) {
        if (id < 0) {
            self.update_meta_data(queue);
        } else {
            self.metadata.iter_mut().for_each(|md| { md[1] = 0; });
            self.metadata[id as usize][1 as usize] = SELECTION_COLOR;
            self.update_meta_data(queue);
        }
        #[cfg(target_arch = "wasm32")]{
            use crate::remote::in_state::select_by_id;
            select_by_id(id);
        }
    }
    fn update_meta_data(&mut self, queue: &Queue) {
        queue.write_buffer(&self.metadata_buffer, 0, bytemuck::cast_slice(&self.metadata));
    }
    fn calculate_offset_pad(curr_mem: u32) -> u32 {
        if (curr_mem < COPY_BYTES_PER_ROW_ALIGNMENT) {
            COPY_BYTES_PER_ROW_ALIGNMENT
        } else {
            let mut count = curr_mem / COPY_BYTES_PER_ROW_ALIGNMENT;
            if (curr_mem % COPY_BYTES_PER_ROW_ALIGNMENT != 0) {
                count = count + 1;
            }
            let memcount = count * COPY_BYTES_PER_ROW_ALIGNMENT;
            memcount
        }
    }
}
unsafe impl Send for GlobalScene {}
unsafe impl Sync for GlobalScene {}
#[derive(Unique)]
pub struct Graphics {
    pub device: Device,
    pub adapter: Adapter,
    pub queue: Queue,
    pub window: Arc<Window>,
    pub surface: Surface<'static>,
    pub surface_config: SurfaceConfiguration,
    pub background_pipe_line: BackGroundPipeLine,
    pub camera: Camera,
    pub mesh_pipe_line: MeshPipeLine,
    pub txt_pipe_line: TxtPipeLine,
}
unsafe impl Send for Graphics {}
unsafe impl Sync for Graphics {}

pub fn init_graphics(world: &World, gr: Graphics) {
    let smaa_target: SmaaTarget = SmaaTarget::new(
        &gr.device,
        &gr.queue,
        gr.surface_config.width,
        gr.surface_config.height,
        gr.surface_config.format,
        SmaaMode::Smaa1X,
    );

    let gs = GlobalState {
        is_right_mouse_pressed: false,
        state: States::Dismiss,
        prev_state: States::Dismiss,
        lraclr_arr: vec![],
        lraclr_arr_reversed: vec![],
        idmaps: HashMap::new(),
        anim_state: AnimState::default(),
        v_up_orign: P_UP_REVERSE,
        instant: Instant::now(),
        dt: 0.0,
        is_next_frame_ready: false,

        smaa_target: smaa_target,
        is_reversed: false,
    };
    let g_scene = GlobalScene::new(
        &gr.device,
        &gr.queue,
        &gs.v_up_orign,
        gr.surface_config.width,
        gr.surface_config.height,
    );

    #[cfg(target_arch = "wasm32")]
    {
        bend_settings(Float32Array::from(
            g_scene.bend_params.params_to_f32vec().as_slice(),
        ));
        let in_cmd = InCmd::new();
        world.add_unique(in_cmd);
    }

    world.add_unique(gr);
    world.add_unique(gs);
    world.add_unique(g_scene);
}
pub fn set_right_mouse_pressed(mut gs: UniqueViewMut<GlobalState>) {
    gs.is_right_mouse_pressed = true;
}
pub fn unset_right_mouse_pressed(mut gs: UniqueViewMut<GlobalState>) {
    gs.is_right_mouse_pressed = false;
}
pub fn mouse_left_pressed(mut gs: UniqueViewMut<GlobalScene>) {}
pub fn mouse_move(pos: PhysicalPosition<f64>, mut gs: UniqueViewMut<GlobalScene>) {
    gs.mouse_x = pos.x;
    gs.mouse_y = pos.y;
}

#[cfg(not(target_arch = "wasm32"))]
pub fn resize_window(
    new_size: PhysicalSize<u32>,
    mut graphics: UniqueViewMut<Graphics>,
    mut gs: UniqueViewMut<GlobalState>,
    mut g_scene: UniqueViewMut<GlobalScene>,
) {
    if new_size.width > 0 && new_size.height > 0 {
        graphics.camera.resize(new_size.width, new_size.height);
        graphics.surface_config.width = new_size.width;
        graphics.surface_config.height = new_size.height;
        graphics.surface.configure(&graphics.device, &graphics.surface_config);
        gs.smaa_target.resize(&graphics.device, new_size.width, new_size.height);
        let arr: [i32; 4] = [new_size.width as i32, new_size.height as i32, 0, 0];
        graphics.background_pipe_line.add_data_buffer = graphics.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("AddData Uniform Buffer").as_str()),
            contents: bytemuck::cast_slice(&arr),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        g_scene.resize(&graphics.device, new_size.width, new_size.height);
    }
}
#[cfg(target_arch = "wasm32")]
pub fn resize_window(
    new_size: PhysicalSize<u32>,
    mut graphics: UniqueViewMut<Graphics>,
    mut gs: UniqueViewMut<GlobalState>,
    mut g_scene: UniqueViewMut<GlobalScene>,
) {
    use winit::platform::web::WindowExtWebSys;
    match graphics.window.canvas() {
        None => {}
        Some(canvas) => {
            graphics.camera.resize(canvas.client_width() as u32, canvas.client_height() as u32);

            graphics.surface_config.width = (canvas.client_width() as f64 * graphics.window.scale_factor()) as u32;
            graphics.surface_config.height = (canvas.client_height() as f64 * graphics.window.scale_factor()) as u32;
            graphics.surface.configure(&graphics.device, &graphics.surface_config);
            gs.smaa_target.resize(
                &graphics.device,
                (canvas.client_width() as f64 * graphics.window.scale_factor()) as u32,
                (canvas.client_height() as f64 * graphics.window.scale_factor()) as u32,
            );
            let arr: [i32; 4] = [
                (canvas.client_width() as f64 * graphics.window.scale_factor()) as i32,
                (canvas.client_height() as f64 * graphics.window.scale_factor()) as i32,
                0,
                0,
            ];
            graphics.background_pipe_line.add_data_buffer = graphics.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(format!("AddData Uniform Buffer").as_str()),
                contents: bytemuck::cast_slice(&arr),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            g_scene.resize(
                &graphics.device,
                (canvas.client_width() as f64 * graphics.window.scale_factor()) as u32,
                (canvas.client_height() as f64 * graphics.window.scale_factor()) as u32,
            );
        }
    }
}
#[inline]
pub fn key_frame(
    mut graphics: UniqueViewMut<Graphics>,
    mut gs: UniqueViewMut<GlobalState>,
    mut g_scene: UniqueViewMut<GlobalScene>,
) {
    gs.check_framerate();
    let next_state: States = {
        match &mut gs.state {
            Dismiss => gs.change_state(States::Dismiss),
            ReadyToLoad((lraclr, is_reset_camera)) => {
                let resetcamera = is_reset_camera.clone();
                gs.lraclr_arr = lraclr.clone();
                gs.lraclr_arr_reversed = cnc::reverse_lraclr(&gs.lraclr_arr);
                let (cyls, tors) = cnc_to_poly(&gs.lraclr_arr, &gs.v_up_orign);
                let (v, i) = all_to_one(&cyls, &tors);
                g_scene.mesh_size = i.len();
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&MESH_ZEROS_I));
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&i));
                graphics.queue.write_buffer(&g_scene.v_buffer_mesh, 0, bytemuck::cast_slice(&v));

                let mut bbx: BoundingBox<cgmath::Point3<f64>> = Default::default();
                cyls.iter().for_each(|cyl| {
                    bbx += cyl.bbx.clone();
                });
                tors.iter().for_each(|tor| {
                    bbx += (tor.bbx.clone());
                });

                if (resetcamera) {
                    graphics.camera.set_tot_bbx(bbx);
                    graphics.camera.set_up_dir(&gs.v_up_orign);
                    graphics.camera.move_camera_to_bbx_limits();
                }

                #[cfg(target_arch = "wasm32")]
                {
                    let lraclr_arr_i32 = LRACLR::to_array(&gs.lraclr_arr);
                    pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(
                        lraclr_arr_i32.as_slice(),
                    ))
                }
                gs.change_state(States::Dismiss)
            }
            ReverseLRACLR => {
                let (cyls, tors) = {
                    if (gs.is_reversed) {
                        gs.is_reversed = false;
                        #[cfg(target_arch = "wasm32")]
                        {
                            let lraclr_arr_i32 = LRACLR::to_array(&gs.lraclr_arr);
                            pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(
                                lraclr_arr_i32.as_slice(),
                            ))
                        }
                        cnc_to_poly(&gs.lraclr_arr, &gs.v_up_orign)
                    } else {
                        gs.is_reversed = true;
                        #[cfg(target_arch = "wasm32")]
                        {
                            let lraclr_arr_i32 = LRACLR::to_array(&gs.lraclr_arr_reversed);
                            pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(
                                lraclr_arr_i32.as_slice(),
                            ))
                        }
                        cnc_to_poly(&gs.lraclr_arr_reversed, &gs.v_up_orign)
                    }
                };

                let (v, i) = all_to_one(&cyls, &tors);
                g_scene.mesh_size = i.len();
                graphics.queue.write_buffer(
                    &g_scene.i_buffer_mesh,
                    0,
                    bytemuck::cast_slice(&MESH_ZEROS_I),
                );
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&i));
                graphics.queue.write_buffer(&g_scene.v_buffer_mesh, 0, bytemuck::cast_slice(&v));
                let mut bbx: BoundingBox<cgmath::Point3<f64>> = Default::default();
                cyls.iter().for_each(|cyl| {
                    bbx += (cyl.bbx.clone());
                });
                tors.iter().for_each(|tor| {
                    let (v_buff, i_buff) = tor.step_vertex_buffer.to_buffers(&graphics.device);
                    bbx += (tor.bbx.clone());
                });
                graphics.camera.set_up_dir(&gs.v_up_orign);
                gs.change_state(States::Dismiss)
            }
            FullAnimate => {
                let (cyls, tors, next_stage) = {
                    if (gs.is_reversed) {
                        cnc::cnc_to_poly_animate(
                            &gs.lraclr_arr_reversed,
                            &gs.anim_state,
                            &gs.v_up_orign,
                            gs.dt,
                            &g_scene.bend_params,
                        )
                    } else {
                        cnc::cnc_to_poly_animate(
                            &gs.lraclr_arr,
                            &gs.anim_state,
                            &gs.v_up_orign,
                            gs.dt,
                            &g_scene.bend_params,
                        )
                    }
                };

                let (v, i) = all_to_one(&cyls, &tors);
                g_scene.mesh_size = i.len();
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&MESH_ZEROS_I));
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&i));
                graphics.queue.write_buffer(&g_scene.v_buffer_mesh, 0, bytemuck::cast_slice(&v));

                #[cfg(target_arch = "wasm32")]
                if (gs.anim_state.op_counter != next_stage.op_counter) {
                    change_bend_step(next_stage.op_counter);
                }
                match gs.anim_state.opcode {
                    0 => {
                        g_scene.dim_x.is_active = true;
                        g_scene.dim_z.is_active = false;
                        g_scene.dim_b.is_active = false;
                        g_scene.dim_x.set_scale(next_stage.lra.pipe_radius);
                        g_scene.dim_x.set_y(next_stage.lra.pipe_radius);
                        g_scene.dim_x.set_x(next_stage.value);
                        g_scene.dim_x.set_pipe_radius(next_stage.lra.pipe_radius);
                        g_scene.dorn.dorn_action(&next_stage, &gs.v_up_orign);
                        gs.anim_state = next_stage;
                        gs.change_state(States::FullAnimate)
                    }
                    1 => {
                        g_scene.dim_x.is_active = false;
                        g_scene.dim_b.is_active = false;
                        g_scene.dim_z.is_active = true;
                        g_scene.dim_z.set_scale(next_stage.lra.pipe_radius);
                        g_scene.dim_z.set_z(next_stage.lra.pipe_radius);
                        g_scene.dim_z.set_r(next_stage.value);
                        g_scene.dim_z.set_pipe_radius(next_stage.lra.pipe_radius);
                        g_scene.dorn.dorn_action(&next_stage, &gs.v_up_orign);
                        gs.anim_state = next_stage;
                        gs.change_state(States::FullAnimate)
                    }
                    2 => {
                        g_scene.dim_b.is_active = true;
                        g_scene.dim_x.is_active = false;
                        g_scene.dim_z.is_active = false;
                        g_scene.dim_b.set_scale(next_stage.lra.pipe_radius);
                        g_scene.dim_b.set_y(next_stage.lra.clr);
                        g_scene.dim_b.set_angle(next_stage.value);
                        g_scene.dim_b.set_pipe_radius(next_stage.lra.pipe_radius);
                        g_scene.dorn.dorn_action(&next_stage, &gs.v_up_orign);
                        gs.anim_state = next_stage;
                        gs.change_state(States::FullAnimate)
                    }
                    4 => {
                        g_scene.dim_x.is_active = false;
                        g_scene.dim_z.is_active = false;
                        g_scene.dim_b.is_active = false;
                        gs.anim_state = AnimState::default();
                        g_scene.dorn.set_dorn_park(&gs.v_up_orign);
                        let (cyls, tors) = cnc_to_poly(&gs.lraclr_arr_reversed, &gs.v_up_orign);

                        let (v, i) = all_to_one(&cyls, &tors);
                        g_scene.mesh_size = i.len();
                        graphics.queue.write_buffer(
                            &g_scene.i_buffer_mesh,
                            0,
                            bytemuck::cast_slice(&MESH_ZEROS_I),
                        );
                        graphics.queue.write_buffer(
                            &g_scene.i_buffer_mesh,
                            0,
                            bytemuck::cast_slice(&i),
                        );
                        graphics.queue.write_buffer(
                            &g_scene.v_buffer_mesh,
                            0,
                            bytemuck::cast_slice(&v),
                        );

                        let mut bbx: BoundingBox<cgmath::Point3<f64>> = Default::default();
                        cyls.iter().for_each(|cyl| {
                            bbx += (cyl.bbx.clone());
                        });
                        tors.iter().for_each(|tor| {
                            bbx += (tor.bbx.clone());
                        });
                        graphics.camera.set_tot_bbx(bbx);
                        graphics.camera.set_up_dir(&gs.v_up_orign);
                        graphics.camera.move_camera_to_bbx_limits();

                        gs.change_state(States::Dismiss)
                    }
                    5 => {
                        gs.anim_state.opcode = 0;
                        graphics.camera.move_to_anim_pos(gs.calculate_total_len(), &gs.v_up_orign);
                        #[cfg(target_arch = "wasm32")]
                        change_bend_step(0);
                        gs.change_state(States::FullAnimate)
                    }
                    _ => {
                        g_scene.dorn.dorn_action(&next_stage, &gs.v_up_orign);
                        gs.anim_state = next_stage;
                        gs.change_state(States::FullAnimate)
                    }
                }
            }
            ChangeDornDir => {
                if (signum(gs.v_up_orign.z) < 0.0) {
                    gs.v_up_orign = P_UP;
                } else {
                    gs.v_up_orign = P_UP_REVERSE;
                }

                let (cyls, tors) = {
                    if (!gs.is_reversed) {
                        #[cfg(target_arch = "wasm32")]
                        {
                            let lraclr_arr_i32 = LRACLR::to_array(&gs.lraclr_arr);
                            pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(
                                lraclr_arr_i32.as_slice(),
                            ))
                        }
                        cnc_to_poly(&gs.lraclr_arr, &gs.v_up_orign)
                    } else {
                        #[cfg(target_arch = "wasm32")]
                        {
                            let lraclr_arr_i32 = LRACLR::to_array(&gs.lraclr_arr_reversed);
                            pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(
                                lraclr_arr_i32.as_slice(),
                            ))
                        }
                        cnc_to_poly(&gs.lraclr_arr_reversed, &gs.v_up_orign)
                    }
                };
                let (v, i) = all_to_one(&cyls, &tors);
                g_scene.mesh_size = i.len();
                graphics.queue.write_buffer(
                    &g_scene.i_buffer_mesh,
                    0,
                    bytemuck::cast_slice(&MESH_ZEROS_I),
                );
                graphics.queue.write_buffer(&g_scene.i_buffer_mesh, 0, bytemuck::cast_slice(&i));
                graphics.queue.write_buffer(&g_scene.v_buffer_mesh, 0, bytemuck::cast_slice(&v));

                let mut bbx: BoundingBox<cgmath::Point3<f64>> = Default::default();
                cyls.iter().for_each(|cyl| {
                    bbx += (cyl.bbx.clone());
                });
                tors.iter().for_each(|tor| {
                    let (v_buff, i_buff) = tor.step_vertex_buffer.to_buffers(&graphics.device);
                    bbx += (tor.bbx.clone());
                });
                graphics.camera.set_up_dir(&gs.v_up_orign);
                gs.change_state(States::Dismiss)
            }
            LoadLRA(v) => {
                let mut lra_cmds: Vec<LRACLR> = vec![];
                if (v.len() % 8 == 0 && !v.is_empty()) {
                    v.chunks(8).for_each(|cmd| {
                        let id1 = cmd[0];
                        let id2 = cmd[1];
                        let l = cmd[2];
                        let lt = cmd[3];
                        let r = cmd[4];
                        let a = cmd[5];
                        let clr = cmd[6];
                        let pipe_radius = cmd[7];
                        let lra_cmd = LRACLR {
                            id1: id1.round() as i32,
                            id2: id2.round() as i32,
                            l: abs(l as f64),
                            lt: Rad::from(Deg(a as f64)).0 * clr as f64,
                            r: r as f64,
                            a: abs(a as f64),
                            clr: abs(clr as f64),
                            pipe_radius: abs(pipe_radius as f64),
                        };
                        lra_cmds.push(lra_cmd);
                    });
                }

                if (lra_cmds.is_empty()) {
                    gs.change_state(States::Dismiss)
                } else {
                    lra_cmds[0].r = 0.0;
                    gs.change_state(ReadyToLoad((lra_cmds, false)))
                }
            }
            States::NewBendParams(params) => {
                g_scene.bend_params.set_params_from_f32vec(&params);
                gs.revert_state()
            }
            States::SelectFromWeb(id) => {
                g_scene.select_by_id(&graphics.queue, id.clone());
                gs.revert_state()
            }
        }
    };
    gs.state = next_state;
}
#[inline]
pub fn render(
    mut graphics: UniqueViewMut<Graphics>,
    mut g_scene: UniqueViewMut<GlobalScene>,
    mut gs: UniqueViewMut<GlobalState>,
) {
    match graphics.surface.get_current_texture() {
        Ok(out) => {
            g_scene.dorn.update(&graphics.device);
            g_scene.dim_x.update(&graphics.device, &graphics.queue, &gs.v_up_orign);
            g_scene.dim_z.update(&graphics.device, &graphics.queue);
            g_scene.dim_b.update(&graphics.device, &graphics.queue, &gs.v_up_orign);

            let mvp = graphics.camera.get_mvp_buffer().clone();
            graphics.queue.write_buffer(
                &g_scene.camera_buffer,
                0,
                bytemuck::cast_slice(&mvp),
            );
            graphics.queue.write_buffer(
                &g_scene.camera_buffer,
                64,
                bytemuck::cast_slice(graphics.camera.get_norm_buffer()),
            );
            graphics.queue.write_buffer(
                &g_scene.camera_buffer,
                128,
                bytemuck::cast_slice(graphics.camera.get_forward_dir_buffer()),
            );
            let gw = graphics.surface_config.width;
            let gh = graphics.surface_config.height;

            let resolution: [f32; 4] = [gw as f32, gh as f32, 0.0, 0.0];
            let light_position: &[f32; 3] = graphics.camera.eye.as_ref();
            let eye_position: &[f32; 3] = graphics.camera.eye.as_ref();
            graphics.queue.write_buffer(
                &g_scene.light_buffer,
                0,
                bytemuck::cast_slice(light_position),
            );
            graphics.queue.write_buffer(
                &g_scene.light_buffer,
                16,
                bytemuck::cast_slice(eye_position),
            );
            graphics.queue.write_buffer(
                &g_scene.light_buffer,
                32,
                bytemuck::cast_slice(&resolution),
            );

            let texture_view_descriptor = TextureViewDescriptor::default();
            let view: TextureView = out.texture.create_view(&texture_view_descriptor);
            let depth_texture: Texture = graphics.device.create_texture(&wgpu::TextureDescriptor {
                size: wgpu::Extent3d {
                    width: gw,
                    height: gh,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                label: None,
                view_formats: &vec![],
            });
            let depth_view: TextureView = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

            let smaa_frame: SmaaFrame = gs.smaa_target.start_frame(&graphics.device, &graphics.queue, &view);

            let mut encoder: CommandEncoder = graphics.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder D"),
            });
            let bg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                layout: &graphics.mesh_pipe_line.mesh_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: g_scene.camera_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: g_scene.light_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: g_scene.material_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: g_scene.metadata_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: g_scene.dorn.scale_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: g_scene.dorn.translate_buffer.as_entire_binding(),
                    },
                ],
                label: Some("Mesh Bind Group"),
            });
            {
                let render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Render Pass1"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &smaa_frame,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(BACKGROUND_COLOR),
                            store: StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: None,
                    occlusion_query_set: None,
                });
            }
            //BACKGROUND
            {
                let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Render Pass 2"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &smaa_frame,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                });
                render_pass.set_pipeline(&graphics.background_pipe_line.render_pipeline);
                render_pass.set_bind_group(
                    0,
                    &graphics.background_pipe_line.mesh_uniform_bind_group,
                    &[],
                );
                render_pass.draw(0..6, 0..1);
            }
            //PIPE_MESHES
            {
                let count = g_scene.mesh_size;
                if (count > 0) {
                    let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("Render Pass 2"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &smaa_frame,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Load,
                                store: StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(
                            wgpu::RenderPassDepthStencilAttachment {
                                view: &depth_view,
                                depth_ops: Some(wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                }),
                                stencil_ops: None,
                            },
                        ),
                        timestamp_writes: None,
                        occlusion_query_set: None,
                    });
                    render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_render_pipeline);
                    render_pass.set_bind_group(0, &bg, &[]);
                    render_pass.set_vertex_buffer(0, g_scene.v_buffer_mesh.slice(..));
                    render_pass.set_index_buffer(
                        g_scene.i_buffer_mesh.slice(..),
                        wgpu::IndexFormat::Uint32,
                    );
                    render_pass.draw_indexed(
                        Range {
                            start: 0,
                            end: count as u32,
                        },
                        0,
                        Range { start: 0, end: 1 },
                    );
                }
            }
            //DORN MESHES
            {
                let count = g_scene.dorn.i_buffer.size() as u64 / mem::size_of::<i32>() as u64;
                let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Render Pass 2"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &smaa_frame,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: None,
                    occlusion_query_set: None,
                });
                render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_render_pipeline);
                render_pass.set_bind_group(0, &bg, &[]);
                render_pass.set_vertex_buffer(0, g_scene.dorn.v_buffer.slice(..));
                render_pass.set_index_buffer(g_scene.dorn.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(
                    Range {
                        start: 0,
                        end: count as u32,
                    },
                    0,
                    Range { start: 0, end: 1 },
                );
            }
            //DIM MESHES
            {
                if (g_scene.dim_x.is_active) {
                    {
                        let count = g_scene.dim_x.i_buffer_x.size() as u64 / mem::size_of::<i32>() as u64;
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_render_pipeline);
                        render_pass.set_bind_group(0, &bg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_x.v_buffer_x.slice(..));
                        render_pass.set_index_buffer(
                            g_scene.dim_x.i_buffer_x.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        render_pass.draw_indexed(
                            Range {
                                start: 0,
                                end: count as u32,
                            },
                            0,
                            Range { start: 0, end: 1 },
                        );
                    }
                    {
                        let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                            layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                            entries: &[
                                wgpu::BindGroupEntry {
                                    binding: 0,
                                    resource: g_scene.camera_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 1,
                                    resource: g_scene.light_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 2,
                                    resource: g_scene.material_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 3,
                                    resource: g_scene.metadata_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 4,
                                    resource: g_scene.dorn.scale_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 5,
                                    resource: g_scene.dorn.translate_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 6,
                                    //resource:BindingResource::TextureView(&graphics.txt_pipe_line.diffuse_texture_view),
                                    resource: BindingResource::TextureView(
                                        &g_scene.dim_x.diffuse_texture_view,
                                    ),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 7,
                                    resource: BindingResource::Sampler(
                                        &g_scene.dim_x.diffuse_sampler,
                                    ),
                                },
                            ],
                            label: Some("Txt Bind Group"),
                        });
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.txt_pipe_line.txt_render_pipeline);
                        render_pass.set_bind_group(0, &tbg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_x.v_txt_buffer.slice(..));
                        render_pass.draw(0..6, 0..1);
                    }
                }

                if (g_scene.dim_z.is_active) {
                    {
                        let count = g_scene.dim_z.i_buffer_x.size() as u64 / mem::size_of::<i32>() as u64;
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_render_pipeline);
                        render_pass.set_bind_group(0, &bg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_z.v_buffer_x.slice(..));
                        render_pass.set_index_buffer(
                            g_scene.dim_z.i_buffer_x.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        render_pass.draw_indexed(
                            Range {
                                start: 0,
                                end: count as u32,
                            },
                            0,
                            Range { start: 0, end: 1 },
                        );
                    }
                    {
                        let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                            layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                            entries: &[
                                wgpu::BindGroupEntry {
                                    binding: 0,
                                    resource: g_scene.camera_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 1,
                                    resource: g_scene.light_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 2,
                                    resource: g_scene.material_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 3,
                                    resource: g_scene.metadata_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 4,
                                    resource: g_scene.dorn.scale_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 5,
                                    resource: g_scene.dorn.translate_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 6,
                                    //resource:BindingResource::TextureView(&graphics.txt_pipe_line.diffuse_texture_view),
                                    resource: BindingResource::TextureView(
                                        &g_scene.dim_z.diffuse_texture_view,
                                    ),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 7,
                                    resource: BindingResource::Sampler(
                                        &g_scene.dim_z.diffuse_sampler,
                                    ),
                                },
                            ],
                            label: Some("Txt Bind Group"),
                        });
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.txt_pipe_line.txt_render_pipeline);
                        render_pass.set_bind_group(0, &tbg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_z.v_txt_buffer.slice(..));
                        render_pass.draw(0..6, 0..1);
                    }
                }

                if (g_scene.dim_b.is_active) {
                    {
                        let count = g_scene.dim_b.i_buffer_x.size() as u64 / mem::size_of::<i32>() as u64;
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_render_pipeline);
                        render_pass.set_bind_group(0, &bg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_b.v_buffer_x.slice(..));
                        render_pass.set_index_buffer(
                            g_scene.dim_b.i_buffer_x.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        render_pass.draw_indexed(
                            Range {
                                start: 0,
                                end: count as u32,
                            },
                            0,
                            Range { start: 0, end: 1 },
                        );
                    }
                    {
                        let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                            layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                            entries: &[
                                wgpu::BindGroupEntry {
                                    binding: 0,
                                    resource: g_scene.camera_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 1,
                                    resource: g_scene.light_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 2,
                                    resource: g_scene.material_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 3,
                                    resource: g_scene.metadata_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 4,
                                    resource: g_scene.dorn.scale_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 5,
                                    resource: g_scene.dorn.translate_buffer.as_entire_binding(),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 6,
                                    //resource:BindingResource::TextureView(&graphics.txt_pipe_line.diffuse_texture_view),
                                    resource: BindingResource::TextureView(
                                        &g_scene.dim_b.diffuse_texture_view,
                                    ),
                                },
                                wgpu::BindGroupEntry {
                                    binding: 7,
                                    resource: BindingResource::Sampler(
                                        &g_scene.dim_b.diffuse_sampler,
                                    ),
                                },
                            ],
                            label: Some("Txt Bind Group"),
                        });
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &smaa_frame,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: Some(
                                wgpu::RenderPassDepthStencilAttachment {
                                    view: &depth_view,
                                    depth_ops: Some(wgpu::Operations {
                                        load: wgpu::LoadOp::Load,
                                        store: StoreOp::Store,
                                    }),
                                    stencil_ops: None,
                                },
                            ),
                            timestamp_writes: None,
                            occlusion_query_set: None,
                        });
                        render_pass.set_pipeline(&graphics.txt_pipe_line.txt_render_pipeline);
                        render_pass.set_bind_group(0, &tbg, &[]);
                        render_pass.set_vertex_buffer(0, g_scene.dim_b.v_txt_buffer.slice(..));
                        render_pass.draw(0..6, 0..1);
                    }
                }
            }

            graphics.queue.submit(iter::once(encoder.finish()));
            smaa_frame.resolve();
            out.present();
            graphics.window.request_redraw();

            if (IS_OFFSCREEN_BUFFER_MAPPED.load(Ordering::Relaxed) && g_scene.is_offscreen_requested) {
                g_scene.is_offscreen_requested = false;
                IS_OFFSCREEN_BUFFER_MAPPED.store(false, Ordering::Relaxed);
                let mut result: Vec<[i32; 4]> = vec![];
                {
                    let slice: BufferSlice = g_scene.offscreen_buffer.slice(..);
                    let view = slice.get_mapped_range();
                    result.extend_from_slice(bytemuck::cast_slice(&view[..]));
                }
                g_scene.offscreen_buffer.unmap();

                let mut rows: Vec<Vec<[i32; 4]>> = vec![];

                let click_aspect_ratio: f32 = g_scene.mouse_x as f32 / graphics.surface_config.width as f32;
                let click_x_compensated: usize = (click_aspect_ratio * g_scene.offscreen_width as f32) as usize;
                result.chunks(g_scene.offscreen_width as usize).for_each(|row| {
                    rows.push(Vec::from(row));
                });
                let selected = &rows[g_scene.mouse_y as usize][click_x_compensated];
                if (selected[3] == 0) {
                    g_scene.select_by_id(&graphics.queue, selected[0]);
                } else {
                    g_scene.unselect_all(&graphics.queue);
                }
            }
        }
        Err(e) => {
            warn!("no surf {:?}", e)
        }
    }
}

pub fn render_selection(
    mut graphics: UniqueViewMut<Graphics>,
    mut g_scene: UniqueViewMut<GlobalScene>,
) {
    if (!g_scene.is_offscreen_requested) {
        let sel_texture_desc = wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: g_scene.offscreen_width,
                height: graphics.surface_config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: TextureFormat::Rgba32Sint,
            usage: wgpu::TextureUsages::COPY_SRC | wgpu::TextureUsages::RENDER_ATTACHMENT,
            label: None,
            view_formats: &[],
        };
        let sel_texture: Texture = graphics.device.create_texture(&sel_texture_desc);
        let sel_texture_view: TextureView = sel_texture.create_view(&Default::default());
        let sel_depth_texture: Texture = graphics.device.create_texture(&wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: g_scene.offscreen_width,
                height: graphics.surface_config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            label: None,
            view_formats: &[],
        });
        let sel_depth_view: TextureView = sel_depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder: CommandEncoder = graphics.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Sel Encoder D"),
        });
        {
            let render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass1"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &sel_texture_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(BACKGROUND_COLOR),
                        store: StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &sel_depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
        }
        //MESH
        {
            let bg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                layout: &graphics.mesh_pipe_line.mesh_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: g_scene.camera_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: g_scene.light_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: g_scene.material_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: g_scene.metadata_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: g_scene.dorn.scale_buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: g_scene.dorn.translate_buffer.as_entire_binding(),
                    },
                ],
                label: Some("Mesh Bind Group"),
            });
            let count = g_scene.mesh_size;
            if (count > 0) {
                let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Render Pass 2"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &sel_texture_view,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &sel_depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Load,
                            store: StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    timestamp_writes: None,
                    occlusion_query_set: None,
                });
                render_pass.set_pipeline(&graphics.mesh_pipe_line.mesh_selection_pipeline);
                render_pass.set_bind_group(0, &bg, &[]);
                render_pass.set_vertex_buffer(0, g_scene.v_buffer_mesh.slice(..));
                render_pass.set_index_buffer(g_scene.i_buffer_mesh.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(
                    Range {
                        start: 0,
                        end: count as u32,
                    },
                    0,
                    Range { start: 0, end: 1 },
                );
            }
        }

        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &sel_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &g_scene.offscreen_buffer,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    // This needs to be a multiple of 256. Normally we would need to pad
                    // it but we here know it will work out anyways.
                    //bytes_per_row: Some((self.mesh_pipeline.offscreen_width * OFFSCREEN_TEXEL_SIZE)),
                    bytes_per_row: Some(g_scene.offscreen_width * OFFSCREEN_TEXEL_SIZE),

                    rows_per_image: Some(graphics.surface_config.height),
                },
            },
            wgpu::Extent3d {
                width: g_scene.offscreen_width,
                height: graphics.surface_config.height,
                depth_or_array_layers: 1,
            },
        );
        graphics.queue.submit(iter::once(encoder.finish()));
        let host_offscreen: BufferSlice = g_scene.offscreen_buffer.slice(..);
        host_offscreen.map_async(wgpu::MapMode::Read, move |result| {
            IS_OFFSCREEN_BUFFER_MAPPED.store(true, Ordering::Relaxed);
        });
        g_scene.is_offscreen_requested = true;
    }
}

#[cfg(target_arch = "wasm32")]
pub fn check_remote(
    mut g_scene: UniqueViewMut<GlobalScene>,
    mut gs: UniqueViewMut<GlobalState>,
    mut cmd: UniqueViewMut<InCmd>,
) {
    match cmd.check_curr_command() {
        States::Dismiss => {}
        ReadyToLoad((v, is_reset_camera)) => {
            gs.v_up_orign = P_UP_REVERSE;
            g_scene.bend_step = 1;
            gs.state = ReadyToLoad((v, is_reset_camera));
        }
        FullAnimate => {
            gs.anim_state = AnimState::default();
            g_scene.bend_step = 1;
            gs.state = FullAnimate
        }
        ReverseLRACLR => {
            if (!gs.lraclr_arr_reversed.is_empty()) {
                g_scene.bend_step = 1;
                gs.state = ReverseLRACLR
            }
        }
        States::ChangeDornDir => {
            g_scene.bend_step = 1;
            gs.state = ChangeDornDir;
        }
        States::LoadLRA(v) => {
            g_scene.bend_step = 1;
            gs.state = LoadLRA(v);
        }
        States::NewBendParams(v) => {
            gs.state = NewBendParams(v);
        }
        States::SelectFromWeb(id) => {
            gs.state = SelectFromWeb(id);
        }
    }
}
pub fn on_keyboard(
    event: KeyEvent,
    mut graphics: UniqueViewMut<Graphics>,
    mut gs: UniqueViewMut<GlobalState>,
    mut g_scene: UniqueViewMut<GlobalScene>,
) {
    match event.physical_key {
        PhysicalKey::Code(KeyCode::F2) => match event.state {
            ElementState::Pressed => {}
            ElementState::Released => {}
        },
        PhysicalKey::Code(KeyCode::F3) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                        g_scene.bend_step = 1;
                        let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                        //let lraclr_arr_i32 = LRACLR::to_array(&lraclr_arr);
                        gs.state = ReadyToLoad((lraclr_arr, true));
                        gs.v_up_orign = P_UP_REVERSE;
                        //let obj_file = ops.all_to_one_obj_bin();
                        //warn!("FILE ANALYZED C {:?}",prerender.steps_data.len());

                    }
                }
            }
        }
        PhysicalKey::Code(KeyCode::F4) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        g_scene.bend_step = 1;
                        let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                        let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                        let lraclr_arr_reversed: Vec<LRACLR> = cnc::reverse_lraclr(&lraclr_arr);
                        gs.state = ReadyToLoad((lraclr_arr, true));
                        gs.v_up_orign = P_UP_REVERSE;

                        //gs.state = ReadyToLoad((prerender,lraclr_arr_reversed));
                        //let obj_file = ops.all_to_one_obj_bin();
                        //warn!("FILE ANALYZED C {:?}",prerender.steps_data.len());

                    }
                }
            }
        }
        PhysicalKey::Code(KeyCode::F5) => match event.state {
            ElementState::Pressed => {}
            ElementState::Released => {
                gs.state = FullAnimate;
            }
        },
        PhysicalKey::Code(KeyCode::F6) => match event.state {
            ElementState::Pressed => {}
            ElementState::Released => {
                g_scene.bend_step = 1;
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/1.stp")).as_slice());
                let stp: Vec<u8> = Vec::from((include_bytes!("../files/13.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/12.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/10.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/9.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/3.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/16.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/a.step")).as_slice());
                let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                let lraclr_arr_reversed: Vec<LRACLR> = cnc::reverse_lraclr(&lraclr_arr);
                gs.state = ReadyToLoad((lraclr_arr, true));
                gs.v_up_orign = P_UP_REVERSE;
            }
        },
        PhysicalKey::Code(KeyCode::F7) => match event.state {
            ElementState::Pressed => {}
            ElementState::Released => {
                g_scene.bend_step = 1;
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/1.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/13.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/12.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/10.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/9.stp")).as_slice());
                let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/D3.step")).as_slice());
                //let stp: Vec<u8> = Vec::from((include_bytes!("../files/a.step")).as_slice());
                let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                let (cyls, tors) = cnc_to_poly(&lraclr_arr, &gs.v_up_orign);
                let file_stp=all_to_stp(&cyls,&tors);
                let path = format!("d:\\pipe_project\\teat.stp");
                match File::create(path) {
                    Ok(file) => {
                        let mut writer = BufWriter::new(file);
                        writer.write_all(&file_stp);
                        writer.flush();
                        warn!("FILE STORED")
                    }
                    Err(_) => {}
                }
            }
        },
        _ => {}
    }
}