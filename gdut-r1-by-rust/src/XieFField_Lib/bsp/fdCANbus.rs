#[derive(Debug, Clone)]
pub struct CanFrame{
    pub id: u32,
    pub data: [u8; 8],
    pub dlc: u8,
    pub is_extended: bool,
}


impl CanFrame {
    pub fn new(id: u32, is_extended: bool) -> Self {
        Self {
            id,
            is_extended: is_extended,
            dlc: 8,
            data: [0u8; 8],
        }
    }
}

