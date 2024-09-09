use std::future::Future;
use std::{iter, mem};
use std::f32::consts::PI;
use std::io::Write;
use std::ops::Range;
use std::rc::Rc;
use std::sync::{Arc, MutexGuard, TryLockResult};
use web_time::{Instant, SystemTime,Duration};
use log::{info, warn};
use smaa::{SmaaMode, SmaaTarget};
//use wasm_bindgen::{JsCast, JsValue};
use web_sys::{Element, HtmlCanvasElement};
use wgpu::{Adapter, BufferSlice, CommandEncoder, Device, Extent3d, Instance, Queue, RenderPass, StoreOp, Surface, SurfaceConfiguration, Texture, TextureFormat, TextureView, TextureViewDescriptor, COPY_BYTES_PER_ROW_ALIGNMENT};
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalSize};
use winit::error::OsError;
use winit::event::{DeviceEvent, DeviceId, ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowAttributes, WindowId};
use crate::device::aux_state::AuxState;
use crate::device::camera::Camera;
use crate::device::materials::Material;
use crate::device::mesh_pipeline::{MeshPipeLine, OFFSCREEN_TEXEL_SIZE};
use crate::device::MeshVertex;
use crate::device::scene::Scene;

use crate::remote::{RemoteCommand, COMMANDS, IS_OFFSCREEN_READY};
#[cfg(target_arch = "wasm32")]
use crate::remote::{pipe_bend_ops, pipe_obj_file};
#[cfg(target_arch = "wasm32")]
use winit::platform::web::WindowExtWebSys;

use crate::trialgo::analyzepl::analyze_bin;
use crate::trialgo::pathfinder::{CncOps, OpElem, LRACLR};

const FRAMERATE:Duration=Duration::from_millis(30);
const BACKGROUND_COLOR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
const MATERIALS_COUNT: usize = 140;

const FRAME_COUNT_DIVIDER: u64 = 100000;
const CANVAS_ID: &str = "canvas3dwindow";

//#[cfg(target_arch = "wasm32")]
#[cfg(not(target_arch = "wasm32"))]
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
#[cfg(not(target_arch = "wasm32"))]
fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output=GState> + 'static {
    let wsize: LogicalSize<u32> = winit::dpi::LogicalSize::new(800, 600);
    let mut window_attrs = Window::default_attributes().with_inner_size(wsize.clone());
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
        let mesh_pipeline: MeshPipeLine = MeshPipeLine::new(&_device, surface_config.format.clone(), wsize.width as i32, wsize.height as i32);


        let scale_factor = rc_window.clone().scale_factor() as f32;
        let w = wsize.width;
        let h = wsize.height;
        let smaa_target: SmaaTarget = SmaaTarget::new(
            &_device,
            &_queue,
            w,
            h,
            surface_config.format.clone(),
            SmaaMode::Smaa1X,
        );


        GState {
            test_counter: 0,
            aux_state: AuxState::new(),
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
            is_offscreen_mapped: false,
            mouse_click_x: 0,
            mouse_click_y: 0,
            scene: Scene::default(),
            smaa_target: smaa_target,
            frame_counter: 0,
            instant: Instant::now(),
        }
    }
}
#[cfg(target_arch = "wasm32")]
async fn create_primary(rc_window: Arc<Window>) -> Option<(Instance, Surface<'static>, Adapter)> {
    warn!("CREATE PRIMARY");
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
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
#[cfg(target_arch = "wasm32")]
async fn create_secondary(rc_window: Arc<Window>) -> Option<(Instance, Surface<'static>, Adapter)> {
    warn!("CREATE secondary");
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::SECONDARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
    let w = rc_window.clone().inner_size().width;
    let h = rc_window.clone().inner_size().height;

    //warn!("CREATE secondary {:?} {:?}", w,h);
    //instance.poll_all(true);
    let surface: Surface = instance.create_surface(rc_window.clone()).unwrap();
    let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
        compatible_surface: Some(&surface), // Some(&surface)
        power_preference: wgpu::PowerPreference::None,
        force_fallback_adapter: false,
    }).await;
    match adapter {
        None => { None }
        Some(adapt) => { Some((instance, surface, adapt)) }
    }
}
/*#[cfg(target_arch = "wasm32")]
fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output=GState> + 'static {
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;

    /*let window: Window ={
        match web_sys::window() {
            None => { panic!("Cant HTML WINDOW") }
            Some(html_window) => {
                match html_window.document() {
                    None => { panic!("Cant GET DOC") }
                    Some(document) => {
                        match document.create_element("canvas") {
                            Err(e) => { panic!("Cant create canvas {:?}", e) }
                            Ok(canvas) => {
                                canvas.set_id("cws_main_p");
                                canvas.set_attribute("width", "100%");
                                canvas.set_attribute("height", "100%");
                                match document.get_element_by_id("wasm3dwindow") {
                                    None => { panic!("NO ID wasm3dwindow") }
                                    Some(dst) => {
                                        match dst.append_with_node_1(canvas.unchecked_ref()) {
                                            Err(e) => { panic!("CANT APPEND CANVAS DST {:?}", e) }
                                            Ok(_) => {
                                                let window_attrs = Window::default_attributes().with_canvas(Some(canvas.unchecked_into()));
                                                match event_loop.create_window(window_attrs) {
                                                    Err(e) => { panic!("CANT CREATE WINDOW {:?}", e) }
                                                    Ok(window) => { window }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    };*/


    /*let window: Window ={
        match web_sys::window() {
            None => { panic!("Cant HTML WINDOW") }
            Some(html_window) => {
                match html_window.document() {
                    None => { panic!("Cant GET DOC") }
                    Some(document) => {
                        match document.get_element_by_id("wasm3dwindow") {
                            None => { panic!("NO ID wasm3dwindow") }
                            Some(dst) => {
                                let sw = dst.client_width();
                                let sh = dst.client_height();
                                info!("HTML ROOM SIZE IS {} {}",sw,sh);



                                match document.create_element("canvas") {
                                    Err(e) => { panic!("Cant create canvas {:?}", e) }
                                    Ok(canvas) => {
                                        canvas.set_id("cws_main_p");
                                        canvas.set_attribute("width", sw.to_string().as_str());
                                        canvas.set_attribute("height", sh.to_string().as_str());
                                        match dst.append_with_node_1(canvas.unchecked_ref()) {
                                            Err(e) => { panic!("CANT APPEND CANVAS DST {:?}", e) }
                                            Ok(_) => {
                                                let ws: PhysicalSize<u32> = PhysicalSize::new(sw as u32, sh as u32);
                                                let window_attrs = Window::default_attributes().with_canvas(Some(canvas.unchecked_into())).with_inner_size(ws);
                                                match event_loop.create_window(window_attrs) {
                                                    Err(e) => { panic!("CANT CREATE WINDOW {:?}", e) }
                                                    Ok(window) => {
                                                        warn!("WINDOW_SIZE {:?} {:?}", window.inner_size().width,window.inner_size().height);
                                                        window
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    };*/

    let window: Window ={
        match web_sys::window() {
            None => { panic!("Cant HTML WINDOW") }
            Some(html_window) => {
                match html_window.document() {
                    None => { panic!("Cant GET DOC") }
                    Some(document) => {
                        match document.get_element_by_id("cws_main_p") {
                            None => { panic!("NO ID cws_main_p") }
                            Some(c) => {

                                let canvas: web_sys::HtmlCanvasElement = c
                                    .dyn_into::<web_sys::HtmlCanvasElement>()
                                    .map_err(|_| ())
                                    .unwrap();


                                let sw = canvas.client_width();
                                let sh = canvas.client_height();
                                info!("HTML ROOM SIZE IS {} {}",sw,sh);
                                let window_attrs: WindowAttributes = Window::default_attributes().with_canvas(Some(canvas));
                                //let ws: PhysicalSize<u32> = PhysicalSize::new(100 as u32, 100 as u32);
                                //let window_attrs = Window::default_attributes().with_inner_size(ws).with_active(true);

                                match event_loop.create_window(window_attrs) {
                                    Err(e) => { panic!("CANT CREATE WINDOW {:?}", e) }
                                    Ok(window) => {
                                        warn!("WINDOW_SIZE2 {:?} {:?}", window.inner_size().width,window.inner_size().height);
                                        window
                                    }
                                }


                            }
                        }
                    }
                }
            }
        }
    };


    let canvas={
        match window.canvas() {
            None => {panic!("NO CANVAS")}
            Some(canvas) => {canvas}
        }
    };

    let _sw = canvas.client_width().clone() as u32;
    let _sh = canvas.client_height().clone() as u32;

    let _winw = window.inner_size().width;
    let _winh = window.inner_size().height;

    if(_winh==0 || _winw==0){
        //panic!("WINDOW SIZE 0");
    }

    let rc_window: Arc<Window> = Arc::new(window);
    async move {
        let (instanse, surface, adapter): (Instance, Surface, Adapter) = {
            match create_primary(rc_window.clone()).await {
                None => {
                    match create_secondary(rc_window.clone()).await {
                        None => { panic!("NOT POSSIBLE TO FOUND SUITABLE GPU") }
                        Some((instanse, surface, adapter)) => {
                            (instanse, surface, adapter)
                        }
                    }
                }
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
        let mut surface_config: SurfaceConfiguration = surface
            .get_default_config(&adapter, _sw as u32, _sh as u32)
            .unwrap();
        surface_config.width=_sw;
        surface_config.height=_sh;
        info!("ADAPTER ATTRIBS {:?}",adapter.get_info());
        info!("SURFACE ATTRIBS {:?}",surface_config);
        surface.configure(&_device, &surface_config);
        info!("SURFACE CONFIGURED");
        let mesh_pipeline: MeshPipeLine = MeshPipeLine::new(&_device, surface_config.format.clone(), _sw as i32, _sh as i32);

        let scale_factor = rc_window.clone().scale_factor() as f32;
        let w = _sw as u32;
        let h = _sh as u32;
        let smaa_target: SmaaTarget = SmaaTarget::new(
            &_device,
            &_queue,
            w,
            h,
            surface_config.format.clone(),
            SmaaMode::Smaa1X,
        );
        GState {
            test_counter: 0,
            is_dirty: false,
            aux_state: AuxState::new(),
            materials: Material::generate_materials(),
            instanse: instanse,
            surface: surface,
            adapter: adapter,
            device: _device,
            queue: _queue,
            rc_window: rc_window,
            scale_factor: scale_factor,
            w: w as u32,
            h: h as u32,
            camera: Camera::default(),
            mesh_pipeline: mesh_pipeline,
            is_offscreen_mapped: false,
            mouse_click_x: 0,
            mouse_click_y: 0,
            scene: Scene::default(),
            smaa_target: smaa_target,
        }
    }
}
*/
#[cfg(target_arch = "wasm32")]
fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output=GState> + 'static {
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let (window, canvas) = match web_sys::window() {
        None => { panic!("Cant WWASM WINDOW") }
        Some(win) => {
            match win.document() {
                None => { panic!("Cant GET DOC") }
                Some(doc) => {
                    match doc.get_element_by_id("wasm3dwindow") {
                        None => { panic!("NO ID wasm3dwindow") }
                        Some(dst) => {
                            let sw = dst.client_width();
                            let sh = dst.client_height();
                            info!("HTML ROOM SIZE IS {} {}",sw,sh);
                            let ws: PhysicalSize<u32> = PhysicalSize::new(sw as u32, sh as u32);
                            let attr = Window::default_attributes().with_inner_size(ws);
                            match event_loop.create_window(attr) {
                                Ok(window) => {
                                    let _scale_factor = window.scale_factor() as f32;
                                    match window.canvas() {
                                        None => { panic!("CANT GET CANVAS") }
                                        Some(canvas) => {
                                            canvas.set_id("cws_main_p");
                                            let canvas_style = canvas.style();
                                            let _ = canvas_style.set_property_with_priority("width", "99%", "");
                                            let _ = canvas_style.set_property_with_priority("height", "99%", "");
                                            match dst.append_child(&canvas).ok() {
                                                None => { panic!("CANT ATTACH CANVAS") }
                                                Some(_n) => {
                                                    warn! {"ATTACHED CANVAS SIZE is :{} {}",canvas.client_width(),canvas.client_height()}
                                                    let _sw = &canvas.client_width();
                                                    let _sh = &canvas.client_height();

                                                    warn! {"Window SIZE is :{:?} {:?}",window.inner_size().width,window.inner_size().height}
                                                    (window, canvas)
                                                }
                                            }
                                        }
                                    }
                                }
                                Err(e) => { panic!("CANT BUILD WINDOWS {:?}", e) }
                            }
                        }
                    }
                }
            }
        }
    };
    let _sw = canvas.client_width().clone() as u32;
    let _sh = canvas.client_height().clone() as u32;
    let rc_window: Arc<Window> = Arc::new(window);
    async move {
        let (instanse, surface, adapter): (Instance, Surface, Adapter) = {
            match create_primary(rc_window.clone()).await {
                None => {
                    match create_secondary(rc_window.clone()).await {
                        None => { panic!("NOT POSSIBLE TO FOUND SUITABLE GPU") }
                        Some((instanse, surface, adapter)) => {
                            (instanse, surface, adapter)
                        }
                    }
                }
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
            .get_default_config(&adapter, _sw as u32, _sh as u32)
            .unwrap();

        info!("ADAPTER ATTRIBS {:?}",adapter.get_info());
        info!("SURFACE ATTRIBS {:?}",surface_config);
        surface.configure(&_device, &surface_config);
        let mesh_pipeline: MeshPipeLine = MeshPipeLine::new(&_device, surface_config.format.clone(), _sw as i32, _sh as i32);

        let scale_factor = rc_window.clone().scale_factor() as f32;
        let w = _sw as u32;
        let h = _sh as u32;
        let smaa_target: SmaaTarget = SmaaTarget::new(
            &_device,
            &_queue,
            w,
            h,
            surface_config.format.clone(),
            SmaaMode::Smaa1X,
        );

        GState {
            test_counter: 0,
            is_dirty: false,
            aux_state: AuxState::new(),
            materials: Material::generate_materials(),
            instanse: instanse,
            surface: surface,
            adapter: adapter,
            device: _device,
            queue: _queue,
            rc_window: rc_window,
            scale_factor: scale_factor,
            w: w as u32,
            h: h as u32,
            camera: Camera::default(),
            mesh_pipeline: mesh_pipeline,
            is_offscreen_mapped: false,
            mouse_click_x: 0,
            mouse_click_y: 0,
            scene: Scene::default(),
            smaa_target: smaa_target,
            frame_counter: 0,
            instant: Instant::now(),
        }
    }
}


enum MaybeGraphics {
    Builder(StateBuilder),
    Graphics(GState),
}

pub enum GEvent {
    State(GState),
    SthngElse(),
}
pub struct GState {
    test_counter: i32,
    aux_state: AuxState,
    is_dirty: bool,
    materials: Vec<Material>,
    instanse: Instance,
    surface: Surface<'static>,
    adapter: Adapter,
    device: Device,
    queue: Queue,
    rc_window: Arc<Window>,
    scale_factor: f32,
    w: u32,
    h: u32,
    camera: Camera,
    mesh_pipeline: MeshPipeLine,
    is_offscreen_mapped: bool,
    mouse_click_x: usize,
    mouse_click_y: usize,
    scene: Scene,
    smaa_target: SmaaTarget,
    frame_counter: u64,
    instant:Instant
}

impl GState {
    #[inline]
    fn render(&mut self) {
        match self.surface.get_current_texture() {
            Ok(out) => {
                self.frame_counter = self.frame_counter + 1;
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
                let smaa_frame = self.smaa_target.start_frame(&self.device, &self.queue, &view);

                let mut encoder: CommandEncoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("Render Encoder D"),
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
                    let background_bind = self.mesh_pipeline.back_ground_pipe_line.create_bind_group(&self.device);
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
                    render_pass.set_pipeline(&self.mesh_pipeline.back_ground_pipe_line.render_pipeline);
                    render_pass.set_bind_group(0, &background_bind, &[]);
                    render_pass.draw(0..6, 0..1);
                }
                //MESH
                {
                    let indx_count = (self.mesh_pipeline.i_buffer.size() / mem::size_of::<i32>() as u64) as u32;
                    if (indx_count > 0) {
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
                        render_pass.set_pipeline(&self.mesh_pipeline.mesh_render_pipeline);
                        render_pass.set_bind_group(0, &bg, &[]);
                        render_pass.set_vertex_buffer(0, self.mesh_pipeline.v_buffer.slice(..));
                        render_pass.set_index_buffer(self.mesh_pipeline.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                        render_pass.draw_indexed(Range { start: 0, end: indx_count }, 0, Range { start: 0, end: 1 });
                    }
                }

                self.queue.submit(iter::once(encoder.finish()));
                smaa_frame.resolve();
                out.present();
            }
            Err(_) => {}
        }

        if (self.is_offscreen_mapped) {
            self.render_to_texture()
        }

    }
    #[inline]
    fn render_to_texture(&mut self) {
        if (!self.is_offscreen_mapped)
        {
            let sel_texture_desc = wgpu::TextureDescriptor {
                size: wgpu::Extent3d {
                    width: self.mesh_pipeline.offscreen_width,
                    height: self.h as u32,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: TextureFormat::Rgba32Sint,
                usage: wgpu::TextureUsages::COPY_SRC
                    | wgpu::TextureUsages::RENDER_ATTACHMENT,
                label: None,
                view_formats: &[],
            };
            let sel_texture: Texture = self.device.create_texture(&sel_texture_desc);

            let sel_texture_view: TextureView = sel_texture.create_view(&Default::default());
            let sel_depth_texture: Texture = self.device.create_texture(&wgpu::TextureDescriptor {
                size: Extent3d {
                    width: self.mesh_pipeline.offscreen_width,
                    height: self.h as u32,
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
            let mut encoder: CommandEncoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
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
                let indx_count = (self.mesh_pipeline.i_buffer.size() / mem::size_of::<i32>() as u64) as u32;
                if (indx_count > 0) {
                    let bg = self.mesh_pipeline.create_bind_group(&self.device);
                    let mut render_pass: RenderPass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("Selection RP1"),
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
                    render_pass.set_pipeline(&self.mesh_pipeline.selection_render_pipeline);
                    render_pass.set_bind_group(0, &bg, &[]);
                    render_pass.set_vertex_buffer(0, self.mesh_pipeline.v_buffer.slice(..));
                    render_pass.set_index_buffer(self.mesh_pipeline.i_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    render_pass.draw_indexed(Range { start: 0, end: indx_count }, 0, Range { start: 0, end: 1 });
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
                    buffer: &self.mesh_pipeline.offscreen_buffer,
                    layout: wgpu::ImageDataLayout {
                        offset: 0,
                        // This needs to be a multiple of 256. Normally we would need to pad
                        // it but we here know it will work out anyways.
                        //bytes_per_row: Some((self.mesh_pipeline.offscreen_width * OFFSCREEN_TEXEL_SIZE)),
                        bytes_per_row: Some(self.mesh_pipeline.offscreen_width * OFFSCREEN_TEXEL_SIZE),

                        rows_per_image: Some(self.h),
                    },
                },
                wgpu::Extent3d {
                    //width: self.mesh_pipeline.offscreen_width,

                    width: self.mesh_pipeline.offscreen_width,
                    height: self.h,
                    depth_or_array_layers: 1,
                },
            );
            self.queue.submit(iter::once(encoder.finish()));

            let host_offscreen: BufferSlice = self.mesh_pipeline.offscreen_buffer.slice(..);
            self.is_offscreen_mapped = true;

            host_offscreen.map_async(wgpu::MapMode::Read, move |result| {
                match result {
                    Ok(_) => {
                        match IS_OFFSCREEN_READY.try_lock() {
                            Ok(mut is_offscreen_ready) => {
                                *is_offscreen_ready = true;
                            }
                            Err(_) => {}
                        }
                    }
                    Err(_) => {}
                }
            });
        } else {
            let is_ready: bool = {
                match IS_OFFSCREEN_READY.try_lock() {
                    Ok(mut is_offscreen_ready) => {
                        let ret: bool = is_offscreen_ready.clone();
                        *is_offscreen_ready = false;
                        ret
                    }
                    Err(_) => { false }
                }
            };
            if (is_ready) {
                let mut result: Vec<[i32; 4]> = vec![];
                {
                    let slice: BufferSlice = self.mesh_pipeline.offscreen_buffer.slice(..);
                    let view = slice.get_mapped_range();
                    result.extend_from_slice(bytemuck::cast_slice(&view[..]));
                }
                self.mesh_pipeline.offscreen_buffer.unmap();
                let mut rows: Vec<Vec<[i32; 4]>> = vec![];
                let row_len = self.mesh_pipeline.offscreen_width as usize;
                let click_aspect_ratio = self.mouse_click_x as f32 / self.w as f32;
                let click_x_compensated = (click_aspect_ratio * row_len as f32) as usize;
                result.chunks(row_len).for_each(|row| {
                    rows.push(Vec::from(row));
                });
                let selected = &rows[self.mouse_click_y][click_x_compensated];
                if (selected[0] != self.scene.selected_id) {
                    self.scene.selected_id = selected[0];
                    //warn!("RESULT SELECTED {:?}",selected);
                    self.mesh_pipeline.select_by_id(&self.device, selected[0]);
                } else {
                    self.mesh_pipeline.unselect_all();
                    self.mesh_pipeline.update_meta_data(&self.device);
                    self.scene.selected_id = 0;
                }
                self.is_offscreen_mapped = false;
            }
        }
    }

    #[inline]
    fn resize(&mut self) {
        self.scale_factor = self.rc_window.clone().scale_factor() as f32;
        let _sw_test = self.rc_window.clone().inner_size().width;
        #[cfg(not(target_arch = "wasm32"))]
        let _sw = self.rc_window.clone().inner_size().width;

        #[cfg(target_arch = "wasm32")]
        let _sw = (self.rc_window.clone().canvas().unwrap().client_width() as f32 * self.scale_factor) as u32;

        #[cfg(not(target_arch = "wasm32"))]
        let _sh = self.rc_window.clone().inner_size().height;
        #[cfg(target_arch = "wasm32")]
        let _sh = (self.rc_window.clone().canvas().unwrap().client_height() as f32 * self.scale_factor) as u32;

        //warn!("CURR {:?} {:?} SF{:?}",_sw_test,_sw, self.rc_window.clone().scale_factor());

        let surface_config: SurfaceConfiguration = self.surface.get_default_config(&self.adapter, _sw as u32, _sh as u32).unwrap(); //info!("SURFACE ATTRIBS {:?}",surface_config);
        self.surface.configure(&self.device, &surface_config);

        self.w = _sw;
        self.h = _sh;
        self.camera.resize(self.w, self.h);
        self.mesh_pipeline.back_ground_pipe_line.resize(&self.device, self.w as i32, self.h as i32);
        self.mesh_pipeline.resize(&self.device, self.w as i32, self.h as i32);
        self.smaa_target.resize(&self.device, self.w, self.h);
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
                                        let (buffer, indxes, bbxs, id_hash, outer_diam): (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>, f64) = ops.to_render_data();
                                        self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                                        self.mesh_pipeline.update_vertexes(&self.device);
                                        self.camera.calculate_tot_bbx(bbxs);
                                        self.camera.move_camera_to_bbx_limits();
                                        let cmds_arr = ops.calculate_lra();
                                        let lraclr_arr: Vec<LRACLR> = ops.calculate_lraclr();
                                        let lraclr_arr_i32 = LRACLR::to_array(&lraclr_arr);

                                        #[cfg(target_arch = "wasm32")]{
                                            pipe_bend_ops(wasm_bindgen_futures::js_sys::Int32Array::from(lraclr_arr_i32.as_slice()));
                                            let obj_file = ops.all_to_one_obj_bin();
                                            pipe_obj_file(wasm_bindgen_futures::js_sys::Uint8Array::from(obj_file.as_slice()));
                                        }

                                        warn!("FILE ANALYZED B{:?}",cmds_arr.len());
                                    }
                                };
                            }
                            RemoteCommand::OnSelectById(id) => {
                                self.mesh_pipeline.select_by_id(&self.device, id);
                            }
                        }
                    }
                }
            }
            Err(_) => { warn!("CANT_LOCK") }
        }
    }
    fn on_keyboard(&mut self, _d: DeviceId, key: KeyEvent, _is_synth: bool, proxy: &EventLoopProxy<GEvent>) {
        match key.physical_key {
            PhysicalKey::Code(KeyCode::F2) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {
                        //let stp: Vec<u8> = Vec::from((include_bytes!("d:/pipe_project/worked/Dapper_6_truba.stp")).as_slice());
                        #[cfg(not(target_arch = "wasm32"))]
                        let stp: Vec<u8> = Vec::from((include_bytes!("d:/pipe_project/worked/ypm_e71042.stp")).as_slice());
                        #[cfg(target_arch = "wasm32")]
                        let stp: Vec<u8> = vec![];
                        match analyze_bin(&stp) {
                            None => {}
                            Some(ops) => {
                                let (buffer, indxes, bbxs, id_hash, outer_diam): (Vec<MeshVertex>, Vec<u32>, Vec<f32>, Vec<u32>, f64) = ops.to_render_data();
                                self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                                self.mesh_pipeline.update_vertexes(&self.device);
                                self.camera.calculate_tot_bbx(bbxs);
                                self.camera.move_camera_to_bbx_limits();
                                let cmds_arr = ops.calculate_lra();
                                let lraclr_arr: Vec<LRACLR> = ops.calculate_lraclr();
                                lraclr_arr.iter().for_each(|cnc| {
                                    warn!("{:?}",cnc);
                                });
                                let obj_file = ops.all_to_one_obj_bin();
                                warn!("FILE ANALYZED C {:?}",cmds_arr.len());
                            }
                        };
                        warn!("F2 Released");
                    }
                }
            }
            PhysicalKey::Code(KeyCode::F3) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {
                        self.mesh_pipeline.select_by_id(&self.device, self.test_counter);
                        self.test_counter = self.test_counter + 1;
                    }
                }
            }
            PhysicalKey::Code(KeyCode::F4) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    assert!(proxy.send_event(GEvent::SthngElse()).is_ok());
                }
                #[cfg(target_arch = "wasm32")]
                {

                    //wasm_bindgen_futures::spawn_local(async move {
                    assert!(proxy.send_event(GEvent::SthngElse()).is_ok());
                    //});
                }
            }
            PhysicalKey::Code(KeyCode::F5) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {
                        #[cfg(not(target_arch = "wasm32"))]
                        let stp: Vec<u8> = Vec::from((include_bytes!("d:/pipe_project/worked/ypm_e71042.stp")).as_slice());
                        #[cfg(target_arch = "wasm32")]
                        let stp: Vec<u8> = vec![];
                        match analyze_bin(&stp) {
                            None => {}
                            Some(ops) => {
                                let (buffer, indxes, bbxs, id_hash, outer_diam) =  ops.generate_unbend_model_from_cl();
                                self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                                self.mesh_pipeline.update_vertexes(&self.device);
                                self.camera.calculate_tot_bbx(bbxs);
                                self.camera.move_camera_to_bbx_limits();
                            }
                        };
                        warn!("F5 Released");

                    }
                }
            }
            PhysicalKey::Code(KeyCode::F6) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {
                        let (buffer, indxes, bbxs, id_hash, outer_diam) =  CncOps::generate_one_cyl();
                        self.mesh_pipeline.step_vertex_buffer.update(buffer, indxes, id_hash);
                        self.mesh_pipeline.update_vertexes(&self.device);
                        self.camera.calculate_tot_bbx(bbxs);
                        self.camera.move_camera_to_bbx_limits();
                    }
                }
            }
            PhysicalKey::Code(KeyCode::F7) => {
                match key.state {
                    ElementState::Pressed => {}
                    ElementState::Released => {

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
        let resolution: [f32; 4] = [self.w as f32, self.h as f32, 0.0, 0.0];
        let light_position: &[f32; 3] = self.camera.eye.as_ref();
        let eye_position: &[f32; 3] = self.camera.eye.as_ref();
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 0, bytemuck::cast_slice(light_position));
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 16, bytemuck::cast_slice(eye_position));
        self.queue.write_buffer(&self.mesh_pipeline.light_buffer, 32, bytemuck::cast_slice(&resolution));
    }
    fn select_by_mouse(&mut self, x: f64, y: f64) {
        self.render_to_texture();
        self.mouse_click_x = x as usize;
        self.mouse_click_y = y as usize;
    }
    pub fn output_image_native(&mut self, image_data: Vec<u8>, texture_dims: (usize, usize)) {
        let path: String = String::from("d:/pipe_project/test.png");
        let mut png_data = Vec::<u8>::with_capacity(image_data.len());
        let mut encoder = png::Encoder::new(
            std::io::Cursor::new(&mut png_data),
            texture_dims.0 as u32,
            texture_dims.1 as u32,
        );
        encoder.set_color(png::ColorType::Rgba);
        let mut png_writer = encoder.write_header().unwrap();
        png_writer.write_image_data(&image_data[..]).unwrap();
        png_writer.finish().unwrap();
        log::info!("PNG file encoded in memory.");

        let mut file = std::fs::File::create(&path).unwrap();
        file.write_all(&png_data[..]).unwrap();
        log::info!("PNG file written to disc as \"{}\".", path);
    }
}

struct StateBuilder {
    pub event_loop_proxy: Option<EventLoopProxy<GEvent>>,
}
impl StateBuilder {
    fn new(event_loop_proxy: EventLoopProxy<GEvent>) -> Self {
        Self {
            event_loop_proxy: Some(event_loop_proxy),
        }
    }
    fn build_and_send(&mut self, event_loop: &ActiveEventLoop) {
        let Some(event_loop_proxy) = self.event_loop_proxy.take() else {
            // event_loop_proxy is already spent - we already constructed Graphics
            return;
        };

        #[cfg(not(target_arch = "wasm32"))]
        {
            let gfx = pollster::block_on(create_graphics(event_loop));
            assert!(event_loop_proxy.send_event(GEvent::State(gfx)).is_ok());
        }
        #[cfg(target_arch = "wasm32")]
        {
            let gfx_fut = create_graphics(event_loop);
            wasm_bindgen_futures::spawn_local(async move {
                let gfx = gfx_fut.await;
                assert!(event_loop_proxy.send_event(GEvent::State(gfx)).is_ok());
            });
        }
    }
}

pub struct Application {
    pub sb: StateBuilder,
    graphics: MaybeGraphics,
    pub x: f64,
    pub y: f64,
}
impl Application {
    pub fn new(event_loop: &EventLoop<GEvent>) -> Self {
        let sb: StateBuilder = StateBuilder::new(event_loop.create_proxy());
        Self {
            sb: sb,
            graphics: MaybeGraphics::Builder(StateBuilder::new(event_loop.create_proxy())),
            x: 0.0,
            y: 0.0,
        }
    }
}

impl ApplicationHandler<GEvent> for Application {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if let MaybeGraphics::Builder(builder) = &mut self.graphics {
            builder.build_and_send(event_loop);
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, mut _event: GEvent) {
        if let GEvent::State(mut event) = _event {
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
        } else {
            match _event {
                GEvent::State(_) => {}
                GEvent::SthngElse() => {
                    warn!("SthngElse");
                }
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
                        match &self.sb.event_loop_proxy {
                            None => {}
                            Some(proxy) => {
                                wstate.on_keyboard(device_id, event, is_synthetic, proxy);
                                wstate.is_dirty=true;
                            }
                        }
                    }
                    WindowEvent::ModifiersChanged(_) => {}
                    WindowEvent::Ime(_) => {}
                    WindowEvent::CursorMoved { device_id, position } => {
                        self.x = position.x;
                        self.y = position.y;
                        wstate.is_dirty=true;
                    }
                    WindowEvent::CursorEntered { .. } => {}
                    WindowEvent::CursorLeft { device_id } => {}

                    WindowEvent::MouseWheel { device_id, delta, phase } => {}
                    WindowEvent::MouseInput { device_id, state, button } => {

                        match button {
                            MouseButton::Left => {
                                match state {
                                    ElementState::Pressed => {
                                        wstate.aux_state.mouse_state.is_left_pressed = true;
                                        wstate.select_by_mouse(self.x, self.y);
                                    }
                                    ElementState::Released => {
                                        wstate.aux_state.mouse_state.is_left_pressed = false;
                                    }
                                }
                            }
                            MouseButton::Right => {
                                match state {
                                    ElementState::Pressed => {
                                        wstate.aux_state.mouse_state.is_right_pressed = true;
                                    }
                                    ElementState::Released => {
                                        wstate.aux_state.mouse_state.is_right_pressed = false;
                                    }
                                }
                            }
                            MouseButton::Middle => {
                                match state {
                                    ElementState::Pressed => {
                                        wstate.aux_state.mouse_state.is_middle_pressed = true;
                                    }
                                    ElementState::Released => {
                                        wstate.aux_state.mouse_state.is_middle_pressed = false;
                                    }
                                }
                            }
                            MouseButton::Back => {
                                match state {
                                    ElementState::Pressed => {}
                                    ElementState::Released => {}
                                }
                            }
                            MouseButton::Forward => {
                                match state {
                                    ElementState::Pressed => {}
                                    ElementState::Released => {}
                                }
                            }
                            MouseButton::Other(_) => {}
                        }
                        wstate.is_dirty=true;
                        wstate.rc_window.clone().request_redraw();

                    }
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
                            wstate.is_dirty = false;

                        wstate.rc_window.clone().request_redraw();
                        //wstate.render_to_texture();
                    }
                }
                if (wstate.is_dirty) {
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
                        if (wstate.aux_state.mouse_state.is_right_pressed) {
                            wstate.camera.update_mouse(delta.0 as f32, delta.1 as f32);
                        }
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
        //warn!("about_to_waitEvent");
        match &mut  self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                wstate.rc_window.clone().request_redraw();
            }
        }
    }
}