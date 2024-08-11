use std::future::Future;
use std::rc::Rc;
use log::{info, warn};
use parking_lot::RwLock;
use wgpu::{Adapter, Device, Instance, Queue, Surface, SurfaceConfiguration};
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::{DeviceEvent, DeviceId, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::platform::web::{WindowAttributesExtWebSys, WindowExtWebSys};
use winit::window::{Window, WindowId};
const CANVAS_ID: &str = "canvas3dwindow";
async fn create_primary(rc_window: Rc<Window>) -> Option<(Instance, Surface<'static>, Adapter)> {
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

async fn create_secondary(rc_window: Rc<Window>) -> Option<(Instance, Surface<'static>, Adapter)> {
    let instance: Instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::SECONDARY,
        flags: Default::default(),
        dx12_shader_compiler: Default::default(),
        gles_minor_version: Default::default(),
    });
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

fn create_graphics(event_loop: &ActiveEventLoop) -> impl Future<Output=WState> + 'static {
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
                            let attr = Window::default_attributes().with_append(true).with_inner_size(ws);
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
    let rc_window: Rc<Window> = Rc::new(window);
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

        let (_device, _queue) = adapter.request_device(&wgpu::DeviceDescriptor {
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
        WState {
            instanse: instanse,
            surface: surface,
            adapter: adapter,
        }
    }
}

struct WStateBuilder {
    event_loop_proxy: Option<EventLoopProxy<WState>>,
}
impl WStateBuilder {
    fn new(event_loop_proxy: EventLoopProxy<WState>) -> Self {
        Self {
            event_loop_proxy: Some(event_loop_proxy),
        }
    }

    fn build_and_send(&mut self, event_loop: &ActiveEventLoop) {
        let Some(event_loop_proxy) = self.event_loop_proxy.take() else {
            // event_loop_proxy is already spent - we already constructed Graphics
            return;
        };
        let gfx_fut = create_graphics(event_loop);
        wasm_bindgen_futures::spawn_local(async move {
            let gfx = gfx_fut.await;
            assert!(event_loop_proxy.send_event(gfx).is_ok());
        });
    }
}

#[allow(dead_code)]
pub struct WState {
    instanse: Instance,
    surface: Surface<'static>,
    adapter: Adapter,
}
enum MaybeGraphics {
    Builder(WStateBuilder),
    Graphics(WState),
}
pub struct Application {
    graphics: MaybeGraphics,
}
impl Application {
    pub fn new(event_loop: &EventLoop<WState>) -> Self {
        Self {
            graphics: MaybeGraphics::Builder(WStateBuilder::new(event_loop.create_proxy())),
        }
    }
}

impl ApplicationHandler<WState> for Application {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if let MaybeGraphics::Builder(builder) = &mut self.graphics {
            builder.build_and_send(event_loop);
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, mut event: WState) {
        self.graphics = MaybeGraphics::Graphics(event);

        match &mut self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {}
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        match &self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                match event {
                    WindowEvent::ActivationTokenDone { .. } => {}
                    WindowEvent::Resized(physical_size) => {}
                    WindowEvent::Moved(_) => {}
                    WindowEvent::CloseRequested => {}
                    WindowEvent::Destroyed => {}
                    WindowEvent::DroppedFile(_) => {}
                    WindowEvent::HoveredFile(_) => {}
                    WindowEvent::HoveredFileCancelled => {}
                    WindowEvent::Focused(_) => {}
                    WindowEvent::KeyboardInput { device_id, event, is_synthetic } => {}
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
                    WindowEvent::RedrawRequested => {}
                }
            }
        }
    }

    fn device_event(&mut self, event_loop: &ActiveEventLoop, device_id: DeviceId, event: DeviceEvent) {
        match &self.graphics {
            MaybeGraphics::Builder(_) => {}
            MaybeGraphics::Graphics(wstate) => {
                match event {
                    DeviceEvent::Added => {}
                    DeviceEvent::Removed => {}
                    DeviceEvent::MouseMotion { delta } => {}
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
                //wstate.window_state.as_ref().unwrap().write().request_redraw(wstate.device_state.as_ref().unwrap().as_ref());
            }
        }
    }
}