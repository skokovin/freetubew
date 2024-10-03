use std::time::Duration;
use log::warn;
use web_time::{Instant, SystemTime};
use crate::trialgo::pathfinder::LRACMD;

const PAUSE:f64 = 500.0;
pub struct DeltaTimeState{
    inst_prew:Instant,
    delta_time:Duration,
    pub curr_value0: f64,
    pub curr_value1: f64,
    pub op: Option<LRACMD>
}

impl DeltaTimeState {
     pub fn new() -> DeltaTimeState {
         Self{
             inst_prew: Instant::now(),
             delta_time: Duration::new(0,0),
             curr_value0: 0.0,
             curr_value1: 0.0,
             op: None,
         }
     }

    pub fn dt(&mut self) -> f64{
        let dt=self.inst_prew.elapsed().as_millis() as f64;
        if(dt>PAUSE){
            warn!("KeyFrame {}",dt);
            self.inst_prew = Instant::now();
            dt
        }else{
            0.0
        }
    }
}