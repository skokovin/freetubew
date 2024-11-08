use std::collections::VecDeque;
use std::sync::Mutex;
use log::{info, warn, Level};
use once_cell::sync::Lazy;
use shipyard::Unique;
use web_sys::js_sys::Uint8Array;
use crate::algo::analyze_bin;
use crate::algo::cnc::LRACLR;
use crate::device::graphics::{Graphics, States};
use crate::device::graphics::States::{ChangeDornDir, FullAnimate, ReadyToLoad, ReverseLRACLR, StandBy};
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::wasm_bindgen;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen_futures::js_sys::Int32Array;

static COMMANDS: Lazy<Mutex<CommandState>> = Lazy::new(|| Mutex::new(CommandState::new()));

struct CommandState {
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
    Reverse,
    ReverseDorn,
}

#[derive(Unique)]
pub struct InCmd {
    lraclr_arr: Vec<LRACLR>,
}
impl InCmd {
    pub fn new() -> InCmd {
        Self {
            lraclr_arr: vec![]
        }
    }
    
    pub fn check_curr_command(&mut self)->States {
        match COMMANDS.try_lock() {
            Ok(mut s) => {
                match s.get_first() {
                    None => {StandBy}
                    Some(command) => {
                        match command {
                            RemoteCommand::OnLoadSTPfile(stp) => {
                                match analyze_bin(&stp) {
                                    None => {StandBy}
                                    Some(mut ops) => {
                                        ReadyToLoad( ops.calculate_lraclr())
                                    }
                                }
                            }
                            RemoteCommand::OnSelectById(id) => {
                                StandBy
                            }
                            RemoteCommand::OnInitBend (stp)=> {
                                StandBy
                            }
                            RemoteCommand::OnDoBend => {
                                FullAnimate
                            }
                            RemoteCommand::Reverse => {
                                ReverseLRACLR
                            }
                            RemoteCommand::ReverseDorn => {
                                ChangeDornDir
                            }
                        }
                    }
                }
            }
            Err(_) => { 
                warn!("CANT_LOCK") ;
                StandBy
            }
        }
    }
}
unsafe impl Send for InCmd {}
unsafe impl Sync for InCmd {}

//TO
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
pub async unsafe fn do_bend() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Error);
    match COMMANDS.lock() {
        Ok(mut m) => {
            m.values.push_back(RemoteCommand::OnDoBend);
        }
        Err(_e) => { warn!("CANT LOCK COMMANDS MEM") }
    }
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub async unsafe fn reverse() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Error);
    match COMMANDS.lock() {
        Ok(mut m) => {
            m.values.push_back(RemoteCommand::Reverse);
        }
        Err(_e) => { warn!("CANT LOCK COMMANDS MEM") }
    }
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub async unsafe fn reverse_dorn() {
    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    let _ = console_log::init_with_level(Level::Error);
    match COMMANDS.lock() {
        Ok(mut m) => {
            m.values.push_back(RemoteCommand::ReverseDorn);
        }
        Err(_e) => { warn!("CANT LOCK COMMANDS MEM") }
    }
}



//////////FROM
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(js_namespace = wvservice)]
extern "C" {
    pub fn pipe_bend_ops(ids: Int32Array);
}