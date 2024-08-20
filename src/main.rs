use env_logger::{Builder, Target};
use log::LevelFilter;
use winit::event_loop::EventLoop;
use crate::device::gstate::{Application, GEvent};

mod libw;
mod device;
mod pipesbend;
mod trialgo;
mod remote;
fn main(){
    let mut builder = Builder::from_default_env();
    builder.target(Target::Stdout);
    builder.filter(None, LevelFilter::Warn).init();

    #[cfg(not(target_arch = "wasm32"))]
    let event_loop: EventLoop<GEvent> = EventLoop::with_user_event().build().unwrap();
    #[cfg(not(target_arch = "wasm32"))]
    let mut app: Application = Application::new(&event_loop);
    #[cfg(not(target_arch = "wasm32"))]
    let _ = event_loop.run_app(&mut app);
}
