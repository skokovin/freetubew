use log::{warn, Level};
use web_sys::wasm_bindgen::prelude::wasm_bindgen;
use winit::event_loop::EventLoop;
use crate::device::wstate::Application;
use crate::device::wstate::WState;
mod device;

#[wasm_bindgen]
#[cfg(target_arch = "wasm32")]
pub async  fn runrust() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Info);
    let event_loop: EventLoop<WState> = EventLoop::with_user_event().build().unwrap();
    let mut app = Application::new(&event_loop);
    let _ = event_loop.run_app(&mut app);
}