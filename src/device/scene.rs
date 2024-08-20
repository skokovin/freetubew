pub struct Scene{
    pub selected_id:i32
}

impl Scene {
    pub fn default()->Self{
        Self{
            selected_id: 0,
        }
    }

}