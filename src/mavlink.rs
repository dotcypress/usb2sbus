use crate::buffer::Buffer;

pub type MavLinkPacket = Buffer<300>;

pub struct MavLinkBuffer {
    packet: MavLinkPacket,
}

impl MavLinkBuffer {
    pub const fn new() -> Self {
        Self {
            packet: MavLinkPacket::new(),
        }
    }

    pub fn append(&mut self, data: &[u8]) -> Option<MavLinkPacket> {
        if self.packet.append(data).is_err() {
            self.packet.clear();
        }

        match self
            .packet
            .iter()
            .position(|stx| *stx == 0xfe || *stx == 0xfd)
        {
            None => {
                self.packet.clear();
                None
            }
            Some(idx) => {
                if idx > 0 {
                    self.packet.pop_front(idx).ok();
                }
                if self.packet.len() < 6 {
                    return None;
                }
                let packet_len = match self.packet[0] {
                    0xfe => self.packet[1] as usize + 8,
                    0xfd if self.packet[2] == 0x01 => self.packet[1] as usize + 25,
                    _ => self.packet[1] as usize + 12,
                };
                self.packet.pop_front(packet_len).ok()
            }
        }
    }
}

impl Default for MavLinkBuffer {
    fn default() -> Self {
        Self::new()
    }
}
