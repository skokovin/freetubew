use std::collections::VecDeque;
use std::sync::Mutex;
use once_cell::sync::Lazy;
use wgpu::BufferSlice;

pub static IS_OFFSCREEN_READY: Lazy<Mutex<bool> >= Lazy::new(|| Mutex::new(false));

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
}