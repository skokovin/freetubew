use std::collections::VecDeque;
use std::sync::Mutex;
use once_cell::sync::Lazy;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::wasm_bindgen;
#[cfg(target_arch = "wasm32")]
use web_sys::js_sys::{Int32Array, Uint8Array};

pub static IS_OFFSCREEN_READY: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

pub static COMMANDS: Lazy<Mutex<CommandState>> = Lazy::new(|| Mutex::new(CommandState::new()));

pub struct CommandState {
    pub values: VecDeque<RemoteCommand>,
}

impl CommandState {
    pub fn new() -> Self {
        Self {
            values: VecDeque::new(),
        }
    }
    pub fn get_first(&mut self) -> Option<RemoteCommand> {
        self.values.remove(0)
    }
}

#[derive(Debug, Clone, PartialEq, )]
pub enum RemoteCommand {
    OnLoadSTPfile((Vec<u8>)),
    OnSelectById(i32),
    OnInitBend((Vec<u8>)),
    OnDoBend,
}

/*#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_namespace = wvservice)]
extern "C" {
    pub fn pipe_bend_ops(ids: Int32Array);
}*/

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_namespace = wvservice)]
extern "C" {
    pub fn pipe_obj_file(ids: Uint8Array);
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_namespace = wvservice)]
extern "C" {
    pub fn pipe_bend_ops(ids: Int32Array);
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_namespace = wvservice)]
extern "C" {
    pub fn selected_by_id(id: i32);
}