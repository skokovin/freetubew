mod device;
mod algo;
mod utils;
//mod libw;

use env_logger::{Builder, Target};
use is_odd::IsOdd;
use log::{LevelFilter};
use winit::application::ApplicationHandler;
use winit::event_loop::{ EventLoop};

use crate::device::app::App;
use crate::device::graphics::Graphics;

fn main() {
    let mut builder = Builder::from_default_env();
    builder.target(Target::Stdout);
    builder.filter(None, LevelFilter::Warn).init();

    let event_loop: EventLoop<Graphics> = EventLoop::with_user_event().build().unwrap();

    let mut app: App = App::new(&event_loop);

    let _ = event_loop.run_app(&mut app);
}
