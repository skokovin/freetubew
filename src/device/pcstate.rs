use std::{iter, mem};
use std::future::Future;
use std::ops::Range;
use std::rc::Rc;
use std::sync::Arc;
use log::{info, warn};
use wgpu::{Adapter, BindGroup, Buffer, BufferAddress, CommandEncoder, Device, Instance, Queue, RenderPass, StoreOp, Surface, SurfaceConfiguration, Texture, TextureView, TextureViewDescriptor};
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalSize};
use winit::event::{DeviceEvent, DeviceId, ElementState, KeyEvent, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::keyboard::{KeyCode, PhysicalKey};

use winit::window::{Window, WindowId};
use crate::device::camera::Camera;
use crate::device::materials::Material;
use crate::device::mesh_pipeline::MeshPipeLine;
use crate::device::{MeshVertex, StepVertexBuffer};
use crate::remote::{RemoteCommand, COMMANDS};
use crate::trialgo::analyzepl::analyze_bin;

const BACKGROUND_COLOR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
const MATERIALS_COUNT: usize = 140;
async fn create_primary(rc_window: Arc<Window>) -> Option<(Instance, Surface<'static>, Adapter)> {
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
    let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
        compatible_surface: None, // Some(&surface)
        power_preference: wgpu::PowerPreference::None,
        force_fallback_adapter: false,
    }).await;
    match adapter {
        None => {
            None
        }
        Some(adapt) => {
            let surface: Surface = instance.create_surface(rc_window.clone()).unwrap();
            Some((instance, surface, adapt))
        }
    }
}

fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output=PcState> + 'static {


    let wsize: LogicalSize<f64> =winit::dpi::LogicalSize::new(800.0, 600.0);
    let mut window_attrs = Window::default_attributes().with_inner_size( wsize.clone());



    let rc_window = Arc::new(event_loop.create_window(window_attrs).unwrap());

    async move {
        let (instanse, surface, adapter): (Instance, Surface, Adapter) = {
            match create_primary(rc_window.clone()).await {
                None => { panic!("NOT POSSIBLE TO FOUND SUITABLE GPU") }
                Some((instanse, surface, adapter)) => {
                    (instanse, surface, adapter)
                }
            }
        };

        let (_device, _queue): (Device, Queue) = adapter.request_device(&wgpu::DeviceDescriptor {
            label: None,
            required_features: Default::default(),
            required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
            memory_hints: Default::default(),
        }, None, ).await.unwrap();
        let surface_config: SurfaceConfiguration = surface
            .get_default_config(&adapter, wsize.width as u32, wsize.height as u32)
            .unwrap();

        info!("ADAPTER ATTRIBS {:?}",adapter.get_info());
        info!("SURFACE ATTRIBS {:?}",surface_config);
        surface.configure(&_device, &surface_config);
        let mesh_pipeline: MeshPipeLine = MeshPipeLine::new(&_device, surface_config.format.clone());



        let scale_factor = rc_window.clone().scale_factor() as f32;
        let w = wsize.width as f32 / scale_factor;
        let h = wsize.height as f32 / scale_factor;

        PcState {
            is_dirty: false,
            materials: Material::generate_materials(),
            instanse: instanse,
            surface: surface,
            adapter: adapter,
            device: _device,
            queue: _queue,
            rc_window: rc_window,
            scale_factor: scale_factor,
            w: w,
            h: h,
            camera: Camera::default(),
            mesh_pipeline: mesh_pipeline,

        }
    }
}


pub struct PcState {
    is_dirty: bool,
    materials: Vec<Material>,
    instanse: Instance,
    surface: Surface<'static>,
    adapter: Adapter,
    device: Device,
    queue: Queue,
    rc_window: Arc<Window>,
    scale_factor: f32,
    w: f32,
    h: f32,
    camera: Camera,
    mesh_pipeline: MeshPipeLine,

}
impl PcState {
    #[inline]
    fn render(&mut self) {
        match self.surface.get_current_texture() {
            Ok(out) => {
                self.update_camera();
                self.update_lights();
                let gw = out.texture.width();
                let gh = out.texture.height();

                let bg = self.mesh_pipeline.create_bind_group(&self.device);

                let texture_view_descriptor = TextureViewDescriptor::default();
                let view: TextureView = out.texture.create_view(&texture_view_descriptor);
                let depth_texture: Texture = self.device.create_texture(&wgpu::TextureDescriptor {
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
                let mut encoder: CommandEncoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("Render Encoder D"),
                });
                {
                    let render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("Render Pass1"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &view,
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

                {
                    let indx_count = (self.mesh_pipeline.i_buffer.size() / mem::size_of::<i32>() as u64) as u32;
                    if (indx_count > 0) {
                        let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Render Pass 2"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &view,
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
                        render_pass.set_pipeline(&self.mesh_pipeline.mesh_render_pipeline);
                        render_pass.set_bind_group(0, &bg, &[]);
                        render_pass.set_vertex_buffer(0, self.mesh_pipeline.v_buffer.slice(..));
                        render_pass.set_index_buffer(self.mesh_pipeline.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(Range { start: 0, end: indx_count }, 0, Range { start: 0, end: 1 });
                    }
                }

                self.queue.submit(iter::once(encoder.finish()));
                out.present();
            }
            Err(_) => {}
        }
    }
    #[inline]
    fn resize(&mut self) {
        self.scale_factor = self.rc_window.clone().scale_factor() as f32;
        let _sw = self.rc_window.clone().inner_size().width;
        let _sh = self.rc_window.clone().inner_size().height;
        let surface_config: SurfaceConfiguration = self.surface.get_default_config(&self.adapter, _sw as u32, _sh as u32).unwrap(); //info!("SURFACE ATTRIBS {:?}",surface_config);
        self.surface.configure(&self.device, &surface_config);

        self.w = _sw as f32 / self.scale_factor;
        self.h = _sh as f32 / self.scale_factor;

        self.camera.resize(self.w,self.h);
        self.is_dirty = true;
    }
    fn check_commands(&mut self) {
        match COMMANDS.try_lock() {
            Ok(mut s) => {
                match s.get_first() {
                    None => {}
                    Some(command) => {
                        match command {
                            RemoteCommand::OnLoadSTPfile(stp) => {
                                warn!("FILE LOADED {:?}",stp.len());
                                match analyze_bin(&stp) {
                                    None => {}
                                    Some(ops) => {
                                        let (buffer, indxes, bbxs, id_hash): (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>) = ops.to_render_data();


                                        self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                                        self.mesh_pipeline.update_vertexes(&self.device);
                                        self.camera.calculate_tot_bbx(bbxs);
                                        self.camera.move_camera_to_bbx_limits();
                                        let cmds_arr = ops.calculate_lra();
                                        let obj_file = ops.all_to_one_obj_bin();
                                        warn!("FILE ANALYZED B{:?}",cmds_arr.len());
                                    }
                                };
                            }
                        }
                    }
                }
            }
            Err(_) => { warn!("CANT_LOCK") }
        }
    }
    fn on_keyboard(&mut self, _d: DeviceId, key: KeyEvent, _is_synth: bool) {
        match key.physical_key {
            PhysicalKey::Code(KeyCode::F2) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {
                        let stp: Vec<u8> = Vec::from((include_bytes!("d:/pipe_project/worked/Dapper_6_truba.stp")).as_slice());

                        match analyze_bin(&stp) {
                            None => {}
                            Some(ops) => {
                                let (buffer, indxes, bbxs, id_hash): (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>) = ops.to_render_data();
                                self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                                self.mesh_pipeline.update_vertexes(&self.device);
                                self.camera.calculate_tot_bbx(bbxs);
                                self.camera.move_camera_to_bbx_limits();
                                let cmds_arr = ops.calculate_lra();
                                let obj_file = ops.all_to_one_obj_bin();
                                warn!("FILE ANALYZED C {:?}",cmds_arr.len());
                            }
                        };
                        warn!("F2 Released");
                    }
                }
            }
            _ => {}
        }
    }
    fn update_camera(&mut self) {
        self.queue.write_buffer(&self.mesh_pipeline.camera_buffer, 0, bytemuck::cast_slice(self.camera.get_mvp_buffer()));
        self.queue.write_buffer(&self.mesh_pipeline.camera_buffer, 64, bytemuck::cast_slice(self.camera.get_norm_buffer()));
        self.queue.write_buffer(&self.mesh_pipeline.camera_buffer, 128, bytemuck::cast_slice(self.camera.get_forward_dir_buffer()));
    }
    fn update_materials(&self) {
        self.queue.write_buffer(&self.mesh_pipeline.material_buffer, 0, bytemuck::cast_slice(&self.materials));
    }
    fn update_lights(&mut self) {
        let resolution: [f32; 4] = [self.w, self.h, 0.0, 0.0];
        let light_position: &[f32; 3] = self.camera.eye.as_ref();
        let eye_position: &[f32; 3] = self.camera.eye.as_ref();
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 0, bytemuck::cast_slice(light_position));
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 16, bytemuck::cast_slice(eye_position));
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 32, bytemuck::cast_slice(&resolution));
    }


}

struct PcStateBuilder {
    event_loop_proxy: Option<EventLoopProxy<PcState>>,
}
impl PcStateBuilder {
    fn new(event_loop_proxy: EventLoopProxy<PcState>) -> Self {
        Self {
            event_loop_proxy: Some(event_loop_proxy),
        }
    }

    fn build_and_send(&mut self, event_loop: &ActiveEventLoop) {
        let Some(event_loop_proxy) = self.event_loop_proxy.take() else {
            // event_loop_proxy is already spent - we already constructed Graphics
            return;
        };

        #[cfg(not(target_arch = "wasm32"))]{
            let gfx = pollster::block_on(create_graphics(event_loop));
            assert!(event_loop_proxy.send_event(gfx).is_ok());
        }

    }
}

enum MaybeGraphics {
    Builder(PcStateBuilder),
    Graphics(PcState),
}
pub struct Application {
    graphics: MaybeGraphics,
}
impl Application {
    pub fn new(event_loop: &EventLoop<PcState>) -> Self {
        Self {
            graphics: MaybeGraphics::Builder(PcStateBuilder::new(event_loop.create_proxy())),
        }
    }
}

impl ApplicationHandler<PcState> for Application {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if let MaybeGraphics::Builder(builder) = &mut self.graphics {
            builder.build_and_send(event_loop);
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, mut event: PcState) {
        event.is_dirty = true;
        self.graphics = MaybeGraphics::Graphics(event);

        match &mut self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                wstate.update_materials();
                wstate.resize();
                wstate.rc_window.clone().request_redraw();
            }
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        match &mut self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                wstate.check_commands();
                match event {
                    WindowEvent::ActivationTokenDone { .. } => {}
                    WindowEvent::Resized(physical_size) => {
                        wstate.resize();
                    }
                    WindowEvent::Moved(_) => {}
                    WindowEvent::CloseRequested => {
                        event_loop.exit();
                    }
                    WindowEvent::Destroyed => {}
                    WindowEvent::DroppedFile(_) => {}
                    WindowEvent::HoveredFile(_) => {}
                    WindowEvent::HoveredFileCancelled => {}
                    WindowEvent::Focused(_) => {}
                    WindowEvent::KeyboardInput { device_id, event, is_synthetic } => {
                        wstate.on_keyboard(device_id, event, is_synthetic);
                    }
                    WindowEvent::ModifiersChanged(_) => {}
                    WindowEvent::Ime(_) => {}
                    WindowEvent::CursorMoved { device_id, position } => {}
                    WindowEvent::CursorEntered { .. } => {}
                    WindowEvent::CursorLeft { device_id } => {}

                    WindowEvent::MouseWheel { device_id, delta, phase } => {}
                    WindowEvent::MouseInput { device_id, state, button } => {}
                    WindowEvent::PinchGesture { .. } => {}
                    WindowEvent::PanGesture { .. } => {}
                    WindowEvent::DoubleTapGesture { .. } => {}
                    WindowEvent::RotationGesture { .. } => {}
                    WindowEvent::TouchpadPressure { .. } => {}
                    WindowEvent::AxisMotion { .. } => {}
                    WindowEvent::Touch(_) => {}
                    WindowEvent::ScaleFactorChanged { .. } => {}
                    WindowEvent::ThemeChanged(_) => {}
                    WindowEvent::Occluded(_) => {}
                    WindowEvent::RedrawRequested => {
                        wstate.render();
                    }
                }
                if (wstate.is_dirty) {
                    wstate.is_dirty = false;
                    wstate.rc_window.clone().request_redraw();
                }
            }
        }
    }

    fn device_event(&mut self, event_loop: &ActiveEventLoop, device_id: DeviceId, event: DeviceEvent) {
        match &mut self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                match event {
                    DeviceEvent::Added => {}
                    DeviceEvent::Removed => {}
                    DeviceEvent::MouseMotion { delta } => {
                        let is_dirty = wstate.camera.update_mouse(delta.0 as f32, delta.1 as f32);
                    }
                    DeviceEvent::MouseWheel { .. } => {}
                    DeviceEvent::Motion { .. } => {}
                    DeviceEvent::Button { .. } => {}
                    DeviceEvent::Key(_) => {}
                }
            }
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        match &self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                wstate.rc_window.clone().request_redraw();
            }
        }
    }
}