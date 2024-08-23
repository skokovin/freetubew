use log::{info, warn, Level};

#[cfg(target_arch = "wasm32")]
use wasm_bindgen_futures::js_sys::Uint8Array;
#[cfg(target_arch = "wasm32")]
use web_sys::wasm_bindgen::prelude::wasm_bindgen;

use winit::event_loop::EventLoop;
use crate::device::gstate::Application;
use crate::device::gstate::GEvent;

#[cfg(target_arch = "wasm32")]
use crate::remote::{RemoteCommand, COMMANDS};

#[cfg(target_arch = "wasm32")]
mod device;
#[cfg(target_arch = "wasm32")]
mod pipesbend;
#[cfg(target_arch = "wasm32")]
mod trialgo;
#[cfg(target_arch = "wasm32")]
mod remote;

#[wasm_bindgen]
#[cfg(target_arch = "wasm32")]
pub async  fn runrust() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Info);
    let event_loop: EventLoop<GEvent> = EventLoop::with_user_event().build().unwrap();
    let mut app = Application::new(&event_loop);
    let _ = event_loop.run_app(&mut app);
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub async unsafe fn read_step_file(arr: Uint8Array) {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Warn);
    warn!("load_step_file");
    let mut handler_v: Vec<u8> = arr.to_vec();
    match COMMANDS.lock() {
        Ok(mut m) => {
            info!("LOAD STEP {:?}",handler_v.len());
            m.values.push_back(RemoteCommand::OnLoadSTPfile(handler_v));

        }
        Err(_e) => { warn!("CANT LOCK COMMANDS MEM") }
    }
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub async unsafe fn select_by_id(id: i32) {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Error);
    match COMMANDS.lock() {
        Ok(mut m) => {
            //info!("SELECT BY ID {:?}",id);
            m.values.push_back(RemoteCommand::OnSelectById(id));
        }
        Err(_e) => { warn!("CANT LOCK COMMANDS MEM") }
    }
}

