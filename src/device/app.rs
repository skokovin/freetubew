use log::{info, warn};
use shipyard::{UniqueView, UniqueViewMut, World};
use std::future::Future;
use std::sync::Arc;
use std::time::Instant;
//use wasm_bindgen::JsCast;
use web_sys::{Element, HtmlCanvasElement};
use wgpu::{
    Adapter, Device, Instance, Queue, Surface, SurfaceConfiguration, SurfaceError, SurfaceTexture,
    TextureFormat,
};

use crate::device::background_pipleine::BackGroundPipeLine;
use crate::device::camera::{update_camera_by_mouse, Camera};
#[cfg(target_arch = "wasm32")]
use crate::device::graphics::check_remote;
use crate::device::graphics::{
    init_graphics, key_frame, on_keyboard, render, resize_window, set_right_mouse_pressed,
    unset_right_mouse_pressed, GlobalState, Graphics,
};
use crate::device::mesh_pipeline::MeshPipeLine;
use crate::device::txt_pipeline::TxtPipeLine;
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalSize};
use winit::error::OsError;
use winit::event::{DeviceEvent, DeviceId, ElementState, MouseButton, StartCause, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::window::{Window, WindowAttributes, WindowId};

pub struct App {
    pub world: World,
    pub event_loop_proxy: Arc<EventLoopProxy<Graphics>>,
    pub is_world_up: bool,
}

impl App {
    pub fn new(el: &EventLoop<Graphics>) -> Self {
        let event_loop_proxy: EventLoopProxy<Graphics> = el.create_proxy();
        Self {
            world: World::new(),
            event_loop_proxy: Arc::new(event_loop_proxy),
            is_world_up: false,
        }
    }
}

impl ApplicationHandler<Graphics> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let proxy = self.event_loop_proxy.clone();
            let gfx: Graphics = pollster::block_on(create_graphics(event_loop));
            assert!(proxy.send_event(gfx).is_ok());
        }
        #[cfg(target_arch = "wasm32")]
        {
            let gfx_fut = create_graphics(event_loop);
            let proxy = self.event_loop_proxy.clone();
            wasm_bindgen_futures::spawn_local(async move {
                let gfx = gfx_fut.await;
                assert!(proxy.send_event(gfx).is_ok());
            });
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: Graphics) {
        self.is_world_up = true;
        let w = event.window.clone();
        init_graphics(&self.world, event);
        w.request_redraw();

        //self.world.add_unique(event);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if (self.is_world_up) {
            match event {
                WindowEvent::CloseRequested => {
                    println!("The close button was pressed; stopping");
                    event_loop.exit();
                }
                WindowEvent::RedrawRequested => {
                    self.world.run(key_frame);
                    self.world.run(render);
                    #[cfg(target_arch = "wasm32")]
                    {
                        self.world.run(check_remote);
                    }

                    self.world.run(|wc: UniqueViewMut<Graphics>| {
                        let w = wc.window.clone();
                        w.request_redraw();
                    });
                }
                WindowEvent::ActivationTokenDone { .. } => {}
                WindowEvent::Resized(physical_size) => {
                    self.world.run_with_data(resize_window, physical_size);
                }
                WindowEvent::Moved(_) => {}
                WindowEvent::Destroyed => {}
                WindowEvent::DroppedFile(_) => {}
                WindowEvent::HoveredFile(_) => {}
                WindowEvent::HoveredFileCancelled => {}
                WindowEvent::Focused(_) => {}
                WindowEvent::KeyboardInput {
                    device_id,
                    event,
                    is_synthetic,
                } => {
                    self.world.run_with_data(on_keyboard, event);
                }
                WindowEvent::ModifiersChanged(_) => {}
                WindowEvent::Ime(_) => {}
                WindowEvent::CursorMoved { .. } => {}
                WindowEvent::CursorEntered { .. } => {}
                WindowEvent::CursorLeft { .. } => {}
                WindowEvent::MouseWheel { .. } => {}
                WindowEvent::MouseInput {
                    device_id,
                    state,
                    button,
                } => match button {
                    MouseButton::Left => match state {
                        ElementState::Pressed => {}
                        ElementState::Released => {}
                    },
                    MouseButton::Right => match state {
                        ElementState::Pressed => {
                            self.world.run(set_right_mouse_pressed);
                        }
                        ElementState::Released => {
                            self.world.run(unset_right_mouse_pressed);
                        }
                    },
                    MouseButton::Middle => match state {
                        ElementState::Pressed => {}
                        ElementState::Released => {}
                    },
                    MouseButton::Back => match state {
                        ElementState::Pressed => {}
                        ElementState::Released => {}
                    },
                    MouseButton::Forward => match state {
                        ElementState::Pressed => {}
                        ElementState::Released => {}
                    },
                    MouseButton::Other(_) => {}
                },
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
            }
        }
    }

    fn device_event(&mut self, event_loop: &ActiveEventLoop, device_id: DeviceId, event: DeviceEvent, ) {
        if (self.is_world_up) {
            match event {
                DeviceEvent::Added => {}
                DeviceEvent::Removed => {}
                DeviceEvent::MouseMotion { delta } => {
                    self.world.run_with_data(update_camera_by_mouse, delta);
                }
                DeviceEvent::Motion { .. } => {}
                DeviceEvent::Button { .. } => {}
                DeviceEvent::Key(_) => {}
                DeviceEvent::MouseWheel { .. } => {}
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output = Graphics> + 'static {
    let wsize: PhysicalSize<u32> = winit::dpi::PhysicalSize::new(800, 600);
    let window_attrs = Window::default_attributes().with_inner_size(wsize.clone());
    let rc_window: Arc<Window> = Arc::new(event_loop.create_window(window_attrs).unwrap());
    async move {
        let (instanse, adapter): (Instance, Adapter) = {
            match create_primary().await {
                None => {
                    panic!("NOT POSSIBLE TO FOUND SUITABLE GPU")
                }
                Some((instanse, adapter)) => (instanse, adapter),
            }
        };

        let (device, queue): (Device, Queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: Default::default(),
                    required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                    memory_hints: Default::default(),
                },
                None,
            )
            .await
            .unwrap();

        let surface: Surface = instanse.create_surface(rc_window.clone()).unwrap();

        let surface_config: SurfaceConfiguration = surface
            .get_default_config(&adapter, wsize.width as u32, wsize.height as u32)
            .unwrap();

        info!("ADAPTER ATTRIBS {:?}", adapter.get_info());
        info!("SURFACE ATTRIBS {:?}", surface_config);
        surface.configure(&device, &surface_config);
        //let window_size: PhysicalSize<u32> = rc_window.inner_size().clone();
        let format: TextureFormat = surface.get_current_texture().unwrap().texture.format();
        let background_pipe_line: BackGroundPipeLine =
            BackGroundPipeLine::new(&device, &format, wsize.width as i32, wsize.height as i32);

        let mesh_pipe_line =
            MeshPipeLine::new(&device, &format, wsize.width as i32, wsize.height as i32);

        let txt_pipe_line = TxtPipeLine::new(
            &device,
            &queue,
            &format,
            wsize.width as i32,
            wsize.height as i32,
        );

        queue.write_buffer(
            &mesh_pipe_line.material_buffer,
            0,
            bytemuck::cast_slice(&mesh_pipe_line.materials),
        );

        Graphics {
            device: device,
            adapter: adapter,
            queue: queue,
            window: rc_window,
            surface: surface,
            surface_config: surface_config,
            //window_size: window_size,
            background_pipe_line: background_pipe_line,
            camera: Camera::default(),
            mesh_pipe_line: mesh_pipe_line,
            txt_pipe_line: txt_pipe_line,
        }
    }
}
#[cfg(not(target_arch = "wasm32"))]
async fn create_primary() -> Option<(Instance, Adapter)> {
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            compatible_surface: None, // Some(&surface)
            power_preference: wgpu::PowerPreference::None,
            force_fallback_adapter: false,
        })
        .await;
    match adapter {
        None => None,
        Some(adapt) => Some((instance, adapt)),
    }
}

#[cfg(target_arch = "wasm32")]
async fn create_primary() -> Option<(Instance, Adapter)> {
    warn!("CREATE PRIMARY");
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            compatible_surface: None, // Some(&surface)
            power_preference: wgpu::PowerPreference::None,
            force_fallback_adapter: false,
        })
        .await;
    match adapter {
        None => None,
        Some(adapt) => Some((instance, adapt)),
    }
}
#[cfg(target_arch = "wasm32")]
async fn create_secondary() -> Option<(Instance, Adapter)> {
    warn!("CREATE secondary");
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::SECONDARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            compatible_surface: None, // Some(&surface)
            power_preference: wgpu::PowerPreference::None,
            force_fallback_adapter: false,
        })
        .await;
    match adapter {
        None => None,
        Some(adapt) => Some((instance, adapt)),
    }
}

#[cfg(target_arch = "wasm32")]
fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output = Graphics> + 'static {
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let window: winit::window::Window = match web_sys::window() {
        None => {
            panic!("Cant WWASM WINDOW")
        }
        Some(win) => match win.document() {
            None => {
                panic!("Cant GET DOC")
            }
            Some(doc) => match doc.get_element_by_id("wasm3dwindow") {
                None => {
                    panic!("NO ID wasm3dwindow")
                }
                Some(dst) => {
                    let sw = dst.client_width();
                    let sh = dst.client_height();

                    use wasm_bindgen::JsCast;
                    let canvas = web_sys::window()
                        .unwrap()
                        .document()
                        .unwrap()
                        .get_element_by_id("cws_main_p")
                        .unwrap()
                        .dyn_into::<web_sys::HtmlCanvasElement>()
                        .ok();
                    warn!("HTML ROOM SIZE IS {} {}", sw, sh);
                    let attr: WindowAttributes =
                        winit::window::Window::default_attributes().with_canvas(canvas);
                    match event_loop.create_window(attr) {
                        Ok(window) => window,
                        Err(e) => {
                            panic!("CANT BUILD WINDOWS {:?}", e)
                        }
                    }
                }
            },
        },
    };

    let rc_window: Arc<winit::window::Window> = Arc::new(window);
    async move {
        match rc_window.canvas() {
            None => {
                panic!("NO CANVAS")
            }
            Some(canvas) => {
                let (instanse, adapter): (Instance, Adapter) = {
                    match create_primary().await {
                        None => match create_secondary().await {
                            None => {
                                panic!("NOT POSSIBLE TO FOUND SUITABLE GPU")
                            }
                            Some((instanse, adapter)) => (instanse, adapter),
                        },
                        Some((instanse, adapter)) => (instanse, adapter),
                    }
                };
                let (device, queue): (Device, Queue) = adapter
                    .request_device(
                        &wgpu::DeviceDescriptor {
                            label: None,
                            required_features: Default::default(),
                            required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                            memory_hints: Default::default(),
                        },
                        None,
                    )
                    .await
                    .unwrap();
                let surface: Surface = instanse.create_surface(rc_window.clone()).unwrap();
                let surface_config: SurfaceConfiguration = surface
                    .get_default_config(
                        &adapter,
                        canvas.client_width() as u32,
                        canvas.client_height() as u32,
                    )
                    .unwrap();
                info!("ADAPTER ATTRIBS {:?}", adapter.get_info());
                info!("SURFACE ATTRIBS {:?}", surface_config);
                surface.configure(&device, &surface_config);

                let format: TextureFormat = surface.get_current_texture().unwrap().texture.format();
                let background_pipe_line: BackGroundPipeLine = BackGroundPipeLine::new(
                    &device,
                    &format,
                    canvas.client_width() as i32,
                    canvas.client_height() as i32,
                );

                let mesh_pipe_line = MeshPipeLine::new(
                    &device,
                    &format,
                    canvas.client_width() as i32,
                    canvas.client_height() as i32,
                );

                let txt_pipe_line = TxtPipeLine::new(
                    &device,
                    &queue,
                    &format,
                    canvas.client_width() as i32,
                    canvas.client_height() as i32,
                );
                queue.write_buffer(
                    &mesh_pipe_line.material_buffer,
                    0,
                    bytemuck::cast_slice(&mesh_pipe_line.materials),
                );
                Graphics {
                    device: device,
                    adapter: adapter,
                    queue: queue,
                    window: rc_window,
                    surface: surface,
                    surface_config: surface_config,
                    background_pipe_line: background_pipe_line,
                    camera: Camera::default(),
                    mesh_pipe_line: mesh_pipe_line,
                    txt_pipe_line: txt_pipe_line,
                }
            }
        }
    }
}
#[cfg(target_arch = "wasm32")]
fn create_graphics_old2(event_loop: &ActiveEventLoop) -> impl Future<Output = Graphics> + 'static {
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let (window, canvas) = match web_sys::window() {
        None => {
            panic!("Cant WWASM WINDOW")
        }
        Some(win) => {
            match win.document() {
                None => {
                    panic!("Cant GET DOC")
                }
                Some(doc) => {
                    match doc.get_element_by_id("wasm3dwindow") {
                        None => {
                            panic!("NO ID wasm3dwindow")
                        }
                        Some(dst) => {
                            let sw = dst.client_width() - 150;
                            let sh = dst.client_height() - 150;
                            warn!("HTML ROOM SIZE IS {} {}", sw, sh);
                            let ws: LogicalSize<u32> = LogicalSize::new(sw as u32, sh as u32);
                            let attr: WindowAttributes =
                                winit::window::Window::default_attributes().with_inner_size(ws);
                            //match event_loop.create_window(attr) {
                            match event_loop.create_window(attr) {
                                Ok(window) => {
                                    let _scale_factor = window.scale_factor() as f32;
                                    match window.canvas() {
                                        None => {
                                            panic!("CANT GET CANVAS")
                                        }
                                        Some(canvas) => {
                                            canvas.set_id("cws_main_p");
                                            let canvas_style = canvas.style();
                                            let _ = canvas_style
                                                .set_property_with_priority("width", "99%", "");
                                            let _ = canvas_style
                                                .set_property_with_priority("height", "99%", "");
                                            match dst.append_child(&canvas).ok() {
                                                None => {
                                                    panic!("CANT ATTACH CANVAS")
                                                }
                                                Some(_n) => {
                                                    warn! {"ATTACHED CANVAS SIZE is :{} {}",canvas.client_width(),canvas.client_height()}
                                                    let _sw = &canvas.client_width();
                                                    let _sh = &canvas.client_height();

                                                    warn! {"Window SIZE is :{:?} {:?}",window.inner_size().width,window.inner_size().height}
                                                    window.request_inner_size(ws);
                                                    (window, canvas)
                                                }
                                            }
                                        }
                                    }
                                }
                                Err(e) => {
                                    panic!("CANT BUILD WINDOWS {:?}", e)
                                }
                            }
                        }
                    }
                }
            }
        }
    };
    let _sw = canvas.client_width().clone() as u32;
    let _sh = canvas.client_height().clone() as u32;
    let _sww = window.inner_size();
    //let _shw = canvas.client_height().clone() as u32;
    let rc_window: Arc<winit::window::Window> = Arc::new(window);
    async move {
        let (instanse, adapter): (Instance, Adapter) = {
            match create_primary().await {
                None => match create_secondary().await {
                    None => {
                        panic!("NOT POSSIBLE TO FOUND SUITABLE GPU")
                    }
                    Some((instanse, adapter)) => (instanse, adapter),
                },
                Some((instanse, adapter)) => (instanse, adapter),
            }
        };

        let (device, queue): (Device, Queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: Default::default(),
                    required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                    memory_hints: Default::default(),
                },
                None,
            )
            .await
            .unwrap();
        let surface: Surface = instanse.create_surface(rc_window.clone()).unwrap();
        let surface_config: SurfaceConfiguration = surface
            .get_default_config(&adapter, _sw as u32, _sh as u32)
            .unwrap();

        info!("ADAPTER ATTRIBS {:?}", adapter.get_info());
        info!("SURFACE ATTRIBS {:?}", surface_config);

        info!(
            "SIZE {:?} {:?} {:?} {:?}",
            _sw, _sh, _sww.width, _sww.height
        );

        surface.configure(&device, &surface_config);
        let window_size: PhysicalSize<u32> = PhysicalSize::new(_sw as u32, _sh as u32); // rc_window.inner_size().clone();
        let format: TextureFormat = surface.get_current_texture().unwrap().texture.format();
        let background_pipe_line: BackGroundPipeLine = BackGroundPipeLine::new(
            &device,
            &format,
            window_size.width as i32,
            window_size.height as i32,
        );

        let mesh_pipe_line = MeshPipeLine::new(
            &device,
            &format,
            window_size.width as i32,
            window_size.height as i32,
        );

        let txt_pipe_line = TxtPipeLine::new(
            &device,
            &queue,
            &format,
            window_size.width as i32,
            window_size.height as i32,
        );

        queue.write_buffer(
            &mesh_pipe_line.material_buffer,
            0,
            bytemuck::cast_slice(&mesh_pipe_line.materials),
        );

        Graphics {
            device: device,
            adapter: adapter,
            queue: queue,
            window: rc_window,
            surface: surface,
            surface_config: surface_config,
            background_pipe_line: background_pipe_line,
            camera: Camera::default(),
            mesh_pipe_line: mesh_pipe_line,
            txt_pipe_line: txt_pipe_line,
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn create_graphics_old(event_loop: &ActiveEventLoop) -> impl Future<Output = Graphics> + 'static {
    use winit::platform::web::WindowAttributesExtWebSys;
    use winit::platform::web::WindowExtWebSys;
    let (window, canvas) = match web_sys::window() {
        None => {
            panic!("Cant WWASM WINDOW")
        }
        Some(win) => {
            match win.document() {
                None => {
                    panic!("Cant GET DOC")
                }
                Some(doc) => {
                    match doc.get_element_by_id("wasm3dwindow") {
                        None => {
                            panic!("NO ID wasm3dwindow")
                        }
                        Some(dst) => {
                            let sw = dst.client_width();
                            let sh = dst.client_height();
                            warn!("HTML ROOM SIZE IS {} {}", sw, sh);
                            //let ws: PhysicalSize<u32> = PhysicalSize::new(sw as u32, sh as u32);
                            //let attr =winit::window::Window::default_attributes().with_inner_size(ws);
                            //match event_loop.create_window(attr) {
                            match event_loop.create_window(Default::default()) {
                                Ok(window) => {
                                    let _scale_factor = window.scale_factor() as f32;
                                    match window.canvas() {
                                        None => {
                                            panic!("CANT GET CANVAS")
                                        }
                                        Some(canvas) => {
                                            canvas.set_id("cws_main_p");
                                            let canvas_style = canvas.style();
                                            let _ = canvas_style
                                                .set_property_with_priority("width", "99%", "");
                                            let _ = canvas_style
                                                .set_property_with_priority("height", "99%", "");
                                            match dst.append_child(&canvas).ok() {
                                                None => {
                                                    panic!("CANT ATTACH CANVAS")
                                                }
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
                                Err(e) => {
                                    panic!("CANT BUILD WINDOWS {:?}", e)
                                }
                            }
                        }
                    }
                }
            }
        }
    };
    let _sw = canvas.client_width().clone() as u32;
    let _sh = canvas.client_height().clone() as u32;
    let rc_window: Arc<winit::window::Window> = Arc::new(window);
    async move {
        let (instanse, adapter): (Instance, Adapter) = {
            match create_primary().await {
                None => match create_secondary().await {
                    None => {
                        panic!("NOT POSSIBLE TO FOUND SUITABLE GPU")
                    }
                    Some((instanse, adapter)) => (instanse, adapter),
                },
                Some((instanse, adapter)) => (instanse, adapter),
            }
        };

        let (device, queue): (Device, Queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: Default::default(),
                    required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                    memory_hints: Default::default(),
                },
                None,
            )
            .await
            .unwrap();
        let surface: Surface = instanse.create_surface(rc_window.clone()).unwrap();
        let surface_config: SurfaceConfiguration = surface
            .get_default_config(&adapter, _sw as u32, _sh as u32)
            .unwrap();

        info!("ADAPTER ATTRIBS {:?}", adapter.get_info());
        info!("SURFACE ATTRIBS {:?}", surface_config);
        surface.configure(&device, &surface_config);

        let format: TextureFormat = surface.get_current_texture().unwrap().texture.format();
        let background_pipe_line: BackGroundPipeLine =
            BackGroundPipeLine::new(&device, &format, _sw as i32, _sh as i32);

        let mesh_pipe_line = MeshPipeLine::new(&device, &format, _sw as i32, _sh as i32);

        let txt_pipe_line = TxtPipeLine::new(&device, &queue, &format, _sw as i32, _sh as i32);

        queue.write_buffer(
            &mesh_pipe_line.material_buffer,
            0,
            bytemuck::cast_slice(&mesh_pipe_line.materials),
        );

        Graphics {
            device: device,
            adapter: adapter,
            queue: queue,
            window: rc_window,
            surface: surface,
            surface_config: surface_config,
            background_pipe_line: background_pipe_line,
            camera: Camera::default(),
            mesh_pipe_line: mesh_pipe_line,
            txt_pipe_line: txt_pipe_line,
        }
    }
}
