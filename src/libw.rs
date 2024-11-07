#![allow(warnings)]

mod device;
mod algo;
mod utils;
mod remote;

use log::Level;
use web_sys::wasm_bindgen::prelude::wasm_bindgen;
use winit::event_loop::{ControlFlow, EventLoop};
use crate::device::app::App;
use crate::device::graphics::Graphics;

#[wasm_bindgen]
#[cfg(target_arch = "wasm32")]
pub async  fn runrust() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Info);
    let event_loop: EventLoop<Graphics> = EventLoop::with_user_event().build().unwrap();
    let mut app: App = App::new(&event_loop);
    let _ = event_loop.run_app(&mut app);
}
