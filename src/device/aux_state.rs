

pub struct AuxState {
    pub mouse_state:MouseButtonsState
}

impl AuxState {
    pub fn new()->Self{
        Self{
            mouse_state: MouseButtonsState::default()
        }
    }
}

pub struct MouseButtonsState{
    pub is_left_pressed:bool,
    pub is_right_pressed:bool,
    pub is_middle_pressed:bool,
}
impl MouseButtonsState {
    pub fn default()->Self{
        Self{
            is_left_pressed: false,
            is_right_pressed: false,
            is_middle_pressed: false,
        }
    }
}


