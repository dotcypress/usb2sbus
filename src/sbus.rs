use crate::buffer::Buffer;

pub const FUTABA_STX: u8 = 0x0f;

pub type FutabaPacket = Buffer<64>;

pub struct FutabaBuffer {
    packet: FutabaPacket,
}

impl FutabaBuffer {
    pub const fn new() -> Self {
        Self {
            packet: FutabaPacket::new(),
        }
    }

    pub fn append(&mut self, data: &[u8]) -> Option<FutabaPacket> {
        if self.packet.append(data).is_err() {
            self.packet.clear();
        }

        match self.packet.iter().position(|b| *b == FUTABA_STX) {
            None => {
                self.packet.clear();
            }
            Some(idx) if idx > 0 => {
                self.packet.pop_front(idx).ok();
            }
            _ => {}
        }
        if self.packet.len() < 25 {
            return None;
        }
        if self.packet[24] != 0x00 {
            self.packet.clear();
            return None;
        }
        self.packet.pop_front(25).ok()
    }
}

impl Default for FutabaBuffer {
    fn default() -> Self {
        Self::new()
    }
}
