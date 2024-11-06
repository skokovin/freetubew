use std::collections::HashMap;
use std::fmt::{Debug, Formatter};
use std::{iter, mem};
use std::ops::Range;
use std::sync::Arc;
use web_time::{Instant, SystemTime};
use log::warn;
use shipyard::{EntitiesViewMut, EntityId, Unique, UniqueViewMut, ViewMut, World};
use smaa::{SmaaFrame, SmaaMode, SmaaTarget};
use truck_base::bounding_box::BoundingBox;
use truck_base::cgmath64::Vector3;
use wgpu::{Adapter, BindGroup, BindingResource, Buffer, CommandEncoder, Device, Queue, RenderPass, StoreOp, Surface, SurfaceConfiguration, Texture, TextureFormat, TextureView, TextureViewDescriptor};
use wgpu::util::DeviceExt;
use winit::dpi::PhysicalSize;
use winit::event::{ElementState, KeyEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::Window;
use crate::algo::cnc::{cnc_to_poly, LRACLR};
use crate::algo::{analyze_bin, cnc, BendToro, MainCylinder};
use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::camera::Camera;
use crate::device::graphics::States::{FullAnimate, ReadyToLoad};
use crate::device::mesh_pipeline::MeshPipeLine;
use crate::device::txt_pipeline::TxtPipeLine;
use crate::utils::dim::{DimB, DimX, DimZ};
use crate::utils::dorn::Dorn;

const STRIGHT_SPEED: f64 = 100.0;
const ROTATE_SPEED: f64 = 10.0;
const ANGLE_SPEED: f64 = 10.0;
const FRAMERATE_SECONDS: f64 = 0.1;
const BACKGROUND_COLOR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
pub enum States {
    StandBy,
    ReadyToLoad((Vec<LRACLR>)),
    FullAnimate,
}
pub struct AnimState {
    pub id: i32,
    pub opcode: usize,
    pub value: f64,
    pub stright_len: f64,
    pub lra: LRACLR,
}
impl AnimState {
    pub fn new(id: i32, opcode: usize, value: f64, stright_len: f64, lra: LRACLR) -> Self {
        Self { id, opcode, value, stright_len, lra }
    }
    pub fn default() -> Self {
        Self {
            id: 0,
            opcode: 0,
            value: 0.0,
            stright_len: 0.0,
            lra: LRACLR::default(),
        }
    }
}
impl Debug for AnimState {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "id {} opcode {} value {}", self.id, self.opcode, self.value)
    }
}
pub struct BendParameters {
    pub stright_speed: f64,
    pub rotate_speed: f64,
    pub angle_speed: f64,
}
pub struct GlobalSceneItem {
    pub e_id: EntityId,
    pub v_buffer: Buffer,
    pub i_buffer: Buffer,

}
#[derive(Unique)]
pub struct GlobalState {
    pub is_right_mouse_pressed: bool,
    pub state: States,
    pub lraclr_arr: Vec<LRACLR>,
    pub lraclr_arr_reversed: Vec<LRACLR>,
    pub cyl_candidates: Vec<MainCylinder>,
    pub tor_candidates: Vec<BendToro>,
    pub idmaps: HashMap<u64, EntityId>,
    pub anim_state: AnimState,
    pub v_up_orign: Vector3,
    pub instant: Instant,
    pub dt: f64,
    pub is_next_frame_ready: bool,
    pub bend_params: BendParameters,
    pub smaa_target: SmaaTarget,
}
impl GlobalState {
    pub fn check_framerate(&mut self) {
        let dt = self.instant.elapsed().as_millis() as f64 / 1000.0;

        if (dt > FRAMERATE_SECONDS) {
            //warn!("Framerate took {} ms", dt);
            self.dt = dt;
            self.instant = Instant::now();
            self.is_next_frame_ready = true
        } else {
            self.is_next_frame_ready = false
        }
    }

    /* pub fn check_framerate_ms(&mut self) {
         let dt = self.instant.elapsed().as_millis() as f64;
         if (dt > FRAMERATE_MILLISECONDS) {
             self.dt = dt * 1000.0;
             self.instant = Instant::now();
             self.is_next_frame_ready = true
         } else {
             self.is_next_frame_ready = false
         }
     }*/
}
unsafe impl Send for GlobalState {}
unsafe impl Sync for GlobalState {}

#[derive(Unique)]
pub struct GlobalScene {
    pub id_buffers: HashMap<u64, GlobalSceneItem>,
    pub u: f64,
    pub bend_step: usize,
    pub dorn: Dorn,
    pub dim_x: DimX,
    pub dim_z: DimZ,
    pub dim_b: DimB,
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
    pub window_size: PhysicalSize<u32>,
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
        gr.window_size.width,
        gr.window_size.height,
        gr.surface_config.format,
        SmaaMode::Smaa1X,
    );

    let gs = GlobalState {
        is_right_mouse_pressed: false,
        state: States::StandBy,
        lraclr_arr: vec![],
        lraclr_arr_reversed: vec![],
        cyl_candidates: vec![],
        tor_candidates: vec![],
        idmaps: HashMap::new(),
        anim_state: AnimState::default(),
        v_up_orign: Vector3::new(0.0, 0.0, -1.0),
        instant: Instant::now(),
        dt: 0.0,
        is_next_frame_ready: false,
        bend_params: BendParameters {
            stright_speed: STRIGHT_SPEED,
            rotate_speed: ROTATE_SPEED,
            angle_speed: ANGLE_SPEED,
        },
        smaa_target: smaa_target,
    };

    let g_scene = GlobalScene {
        id_buffers: HashMap::new(),
        u: 0.0,
        bend_step: 1,
        dorn: Dorn::new(&gr.device, &gs.v_up_orign),
        dim_x: DimX::new(&gr.device),
        dim_z: DimZ::new(&gr.device),
        dim_b: DimB::new(&gr.device),
    };

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

pub fn resize_window(new_size: PhysicalSize<u32>, mut graphics: UniqueViewMut<Graphics>, mut gs: UniqueViewMut<GlobalState>,) {
    if new_size.width > 0 && new_size.height > 0 {
        graphics.camera.resize(new_size.width, new_size.height);
        graphics.window_size = new_size;
        graphics.surface_config.width = new_size.width;
        graphics.surface_config.height = new_size.height;
        graphics.surface.configure(&graphics.device, &graphics.surface_config);
        gs.smaa_target.resize(&graphics.device,new_size.width,new_size.height);
        let arr: [i32; 4] = [new_size.width as i32, new_size.height as i32, 0, 0];
        graphics.background_pipe_line.add_data_buffer = graphics.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(format!("AddData Uniform Buffer").as_str()),
            contents: bytemuck::cast_slice(&arr),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    }
}

pub fn key_frame(mut graphics: UniqueViewMut<Graphics>,
                 mut gs: UniqueViewMut<GlobalState>,
                 mut g_scene: UniqueViewMut<GlobalScene>,
                 mut entities: EntitiesViewMut,
                 mut cyls_comps: ViewMut<MainCylinder>,
                 mut tor_comps: ViewMut<BendToro>, ) {
    gs.check_framerate();
    if (gs.is_next_frame_ready) {
        let next_state: States = {
            match &mut gs.state {
                States::StandBy => { States::StandBy }
                States::ReadyToLoad((lraclr)) => {
                    cyls_comps.clear();
                    tor_comps.clear();
                    let lraclr_cloned = lraclr.clone();
                    gs.lraclr_arr = lraclr_cloned.clone();
                    gs.lraclr_arr_reversed = cnc::reverse_lraclr(lraclr_cloned.clone());
                    let (cyls, tors) = cnc_to_poly(&gs.lraclr_arr, &gs.v_up_orign);
                    gs.tor_candidates = tors;
                    gs.cyl_candidates = cyls;
                    let mut bbx: BoundingBox<cgmath::Point3<f64>> = Default::default();
                    gs.cyl_candidates.iter_mut().for_each(|cyl| {
                        let (v_buff, i_buff) = cyl.step_vertex_buffer.to_buffers(&graphics.device);
                        let e_id: EntityId = entities.add_entity(&mut cyls_comps, cyl.clone());
                        g_scene.id_buffers.insert(
                            cyl.id,
                            GlobalSceneItem {
                                e_id,
                                v_buffer: v_buff,
                                i_buffer: i_buff,
                            },
                        );
                        bbx += (cyl.bbx.clone());
                    });
                    gs.tor_candidates.iter().for_each(|tor| {
                        let (v_buff, i_buff) = tor.step_vertex_buffer.to_buffers(&graphics.device);
                        let e_id: EntityId = entities.add_entity(&mut tor_comps, tor.clone());
                        g_scene.id_buffers.insert(
                            tor.id,
                            GlobalSceneItem {
                                e_id,
                                v_buffer: v_buff,
                                i_buffer: i_buff,
                            },
                        );

                        bbx += (tor.bbx.clone());
                    });
                    graphics.camera.set_tot_bbx(bbx);
                    graphics.camera.move_camera_to_bbx_limits();
                    States::StandBy
                    //warn!("BBX {:?}",bbx);
                }

                States::FullAnimate => {
                    let (cyls, tors, next_stage) =
                        cnc::cnc_to_poly_v(&gs.lraclr_arr, &gs.anim_state, &gs.v_up_orign, gs.dt, &gs.bend_params);
                    cyls_comps.clear();
                    tor_comps.clear();
                    cyls.iter().for_each(|cyl| {
                        let (v_buff, i_buff) = cyl.step_vertex_buffer.to_buffers(&graphics.device);
                        let e_id: EntityId = entities.add_entity(&mut cyls_comps, cyl.clone());
                        g_scene.id_buffers.insert(
                            cyl.id,
                            GlobalSceneItem {
                                e_id,
                                v_buffer: v_buff,
                                i_buffer: i_buff,
                            },
                        );
                    });
                    tors.iter().for_each(|tor| {
                        let (v_buff, i_buff) = tor.step_vertex_buffer.to_buffers(&graphics.device);
                        let e_id: EntityId = entities.add_entity(&mut tor_comps, tor.clone());
                        g_scene.id_buffers.insert(
                            tor.id,
                            GlobalSceneItem {
                                e_id,
                                v_buffer: v_buff,
                                i_buffer: i_buff,
                            },
                        );
                    });

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
                            States::FullAnimate
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
                            States::FullAnimate
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
                            States::FullAnimate
                        }

                        4 => {
                            g_scene.dim_x.is_active = false;
                            g_scene.dim_z.is_active = false;
                            g_scene.dim_b.is_active = false;
                            gs.anim_state = AnimState::default();
                            let lra = gs.lraclr_arr.clone();
                            g_scene.dorn.set_dorn_park(&gs.v_up_orign);
                            States::ReadyToLoad(lra)
                        }
                        _ => {
                            g_scene.dorn.dorn_action(&next_stage, &gs.v_up_orign);
                            gs.anim_state = next_stage;
                            States::FullAnimate
                        }
                    }
                }
            }
        };
        gs.state = next_state;
    }
}
pub fn render(mut graphics: UniqueViewMut<Graphics>,
              mut g_scene: UniqueViewMut<GlobalScene>,
              mut gs: UniqueViewMut<GlobalState>,
              mut cyls_comps: ViewMut<MainCylinder>,
              mut tor_comps: ViewMut<BendToro>, ) {
    
    if (gs.is_next_frame_ready) {
        match graphics.surface.get_current_texture() {
            Ok(out) => {
                g_scene.dorn.update(&graphics.device);
                g_scene.dim_x.update(&graphics.device, &graphics.queue, &gs.v_up_orign);
                g_scene.dim_z.update(&graphics.device, &graphics.queue);
                g_scene.dim_b.update(&graphics.device, &graphics.queue, &gs.v_up_orign);

                let mvp = graphics.camera.get_mvp_buffer().clone();
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.camera_buffer, 0, bytemuck::cast_slice(&mvp));
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.camera_buffer, 64, bytemuck::cast_slice(graphics.camera.get_norm_buffer()));
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.camera_buffer, 128, bytemuck::cast_slice(graphics.camera.get_forward_dir_buffer()));
                //graphics.queue.write_buffer(&graphics.mesh_pipe_line.material_buffer, 0, bytemuck::cast_slice(&&graphics.mesh_pipe_line.materials));

                let resolution: [f32; 4] = [graphics.window_size.width as f32, graphics.window_size.height as f32, 0.0, 0.0];
                let light_position: &[f32; 3] = graphics.camera.eye.as_ref();
                let eye_position: &[f32; 3] = graphics.camera.eye.as_ref();
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.light_buffer, 0, bytemuck::cast_slice(light_position));
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.light_buffer, 16, bytemuck::cast_slice(eye_position));
                graphics.queue.write_buffer(&graphics.mesh_pipe_line.light_buffer, 32, bytemuck::cast_slice(&resolution));


                let gw = out.texture.width();
                let gh = out.texture.height();
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

                let smaa_frame: SmaaFrame =  gs.smaa_target.start_frame(&graphics.device, &graphics.queue, &view);

                let mut encoder: CommandEncoder = graphics.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("Render Encoder D"),
                });
                let bg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                    layout: &graphics.mesh_pipe_line.mesh_bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: graphics.mesh_pipe_line.camera_buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: graphics.mesh_pipe_line.light_buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: graphics.mesh_pipe_line.material_buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 3,
                            resource: graphics.mesh_pipe_line.metadata_buffer.as_entire_binding(),
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
                    render_pass.set_bind_group(0, &graphics.background_pipe_line.mesh_uniform_bind_group, &[]);
                    render_pass.draw(0..6, 0..1);
                }
                //CYL MESHES
                {
                    cyls_comps.as_slice().iter().for_each(|cyl| {
                        match g_scene.id_buffers.get(&cyl.id) {
                            None => {}
                            Some(rd) => {
                                let count = rd.i_buffer.size() as u64 / mem::size_of::<i32>() as u64;
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
                                render_pass.set_vertex_buffer(0, rd.v_buffer.slice(..));
                                render_pass.set_index_buffer(rd.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                                render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
                                //warn!("count {:?} {:?}",cyl.id,count);
                            }
                        }
                    });
                }
                //TOR MESHES
                {
                    tor_comps.as_slice().iter().for_each(|cyl| {
                        match g_scene.id_buffers.get(&cyl.id) {
                            None => {}
                            Some(rd) => {
                                let count = rd.i_buffer.size() as u64 / mem::size_of::<i32>() as u64;
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
                                render_pass.set_vertex_buffer(0, rd.v_buffer.slice(..));
                                render_pass.set_index_buffer(rd.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                                render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
                                //warn!("count {:?} {:?}",cyl.id,count);
                            }
                        }
                    });
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
                    render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
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
                            render_pass.set_vertex_buffer(0, g_scene.dim_x.v_buffer_x.slice(..));
                            render_pass.set_index_buffer(g_scene.dim_x.i_buffer_x.slice(..), wgpu::IndexFormat::Uint32);
                            render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
                        }
                        {
                            let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                                layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                                entries: &[
                                    wgpu::BindGroupEntry {
                                        binding: 0,
                                        resource: graphics.mesh_pipe_line.camera_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 1,
                                        resource: graphics.mesh_pipe_line.light_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 2,
                                        resource: graphics.mesh_pipe_line.material_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 3,
                                        resource: graphics.mesh_pipe_line.metadata_buffer.as_entire_binding(),
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
                                        resource: BindingResource::TextureView(&g_scene.dim_x.diffuse_texture_view),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 7,
                                        resource: BindingResource::Sampler(&g_scene.dim_x.diffuse_sampler),
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
                            render_pass.set_vertex_buffer(0, g_scene.dim_z.v_buffer_x.slice(..));
                            render_pass.set_index_buffer(g_scene.dim_z.i_buffer_x.slice(..), wgpu::IndexFormat::Uint32);
                            render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
                        }
                        {
                            let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                                layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                                entries: &[
                                    wgpu::BindGroupEntry {
                                        binding: 0,
                                        resource: graphics.mesh_pipe_line.camera_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 1,
                                        resource: graphics.mesh_pipe_line.light_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 2,
                                        resource: graphics.mesh_pipe_line.material_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 3,
                                        resource: graphics.mesh_pipe_line.metadata_buffer.as_entire_binding(),
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
                                        resource: BindingResource::TextureView(&g_scene.dim_z.diffuse_texture_view),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 7,
                                        resource: BindingResource::Sampler(&g_scene.dim_z.diffuse_sampler),
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
                            render_pass.set_vertex_buffer(0, g_scene.dim_b.v_buffer_x.slice(..));
                            render_pass.set_index_buffer(g_scene.dim_b.i_buffer_x.slice(..), wgpu::IndexFormat::Uint32);
                            render_pass.draw_indexed(Range { start: 0, end: count as u32 }, 0, Range { start: 0, end: 1 });
                        }
                        {
                            let tbg: BindGroup = graphics.device.create_bind_group(&wgpu::BindGroupDescriptor {
                                layout: &graphics.txt_pipe_line.txt_bind_group_layout,
                                entries: &[
                                    wgpu::BindGroupEntry {
                                        binding: 0,
                                        resource: graphics.mesh_pipe_line.camera_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 1,
                                        resource: graphics.mesh_pipe_line.light_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 2,
                                        resource: graphics.mesh_pipe_line.material_buffer.as_entire_binding(),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 3,
                                        resource: graphics.mesh_pipe_line.metadata_buffer.as_entire_binding(),
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
                                        resource: BindingResource::TextureView(&g_scene.dim_b.diffuse_texture_view),
                                    },
                                    wgpu::BindGroupEntry {
                                        binding: 7,
                                        resource: BindingResource::Sampler(&g_scene.dim_b.diffuse_sampler),
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
            }
            Err(e) => {warn!("no surf {:?}",e)}
        }
    }
}
pub fn on_keyboard(event: KeyEvent,
                   mut graphics: UniqueViewMut<Graphics>,
                   mut gs: UniqueViewMut<GlobalState>,
                   mut g_scene: UniqueViewMut<GlobalScene>) {
    match event.physical_key {
        PhysicalKey::Code(KeyCode::F2) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {}
            }
        }
        PhysicalKey::Code(KeyCode::F3) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    //#[cfg(not(target_arch = "wasm32"))]
                    {
                        let stp: Vec<u8> = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                        match analyze_bin(&stp) {
                            None => {}
                            Some(mut ops) => {
                                g_scene.bend_step = 1;
                                let lraclr_arr: Vec<LRACLR> = ops.calculate_lraclr();
                                //let lraclr_arr_i32 = LRACLR::to_array(&lraclr_arr);
                                gs.state = ReadyToLoad((lraclr_arr));
                                gs.v_up_orign = Vector3::new(0.0, 0.0, -1.0);
                                //let obj_file = ops.all_to_one_obj_bin();
                                //warn!("FILE ANALYZED C {:?}",prerender.steps_data.len());
                            }
                        };
                    }
                }
            }
        }
        PhysicalKey::Code(KeyCode::F4) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    //#[cfg(not(target_arch = "wasm32"))]
                    {
                        g_scene.bend_step = 1;
                        let stp: Vec<u8> = Vec::from((include_bytes!("../files/9.stp")).as_slice());
                        match analyze_bin(&stp) {
                            None => {}
                            Some(mut ops) => {
                                let lraclr_arr: Vec<LRACLR> = ops.calculate_lraclr();
                                let lraclr_arr_reversed: Vec<LRACLR> = cnc::reverse_lraclr(lraclr_arr.clone());
                                gs.state = ReadyToLoad(lraclr_arr_reversed);
                                gs.v_up_orign = Vector3::new(0.0, 0.0, -1.0);


                                //gs.state = ReadyToLoad((prerender,lraclr_arr_reversed));
                                //let obj_file = ops.all_to_one_obj_bin();
                                //warn!("FILE ANALYZED C {:?}",prerender.steps_data.len());
                            }
                        };
                    }
                }
            }
        }
        PhysicalKey::Code(KeyCode::F5) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    gs.state = FullAnimate;
                }
            }
        }
        PhysicalKey::Code(KeyCode::F6) => {
            match event.state {
                ElementState::Pressed => {}
                ElementState::Released => {
                    g_scene.dim_x.is_active = true;
                    g_scene.dim_x.set_scale(300.0);
                }
            }
        }
        _ => {}
    }
}