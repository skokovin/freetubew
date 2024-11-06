use shipyard::{track, Component};

pub struct BendPipe{

}

impl Component for BendPipe {
    // We'll come back to this in a later chapter
    type Tracking = track::Untracked;
}