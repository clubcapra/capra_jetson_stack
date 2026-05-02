use std::io;
use std::net::{SocketAddr, UdpSocket};

pub struct Input {
    pub socket: UdpSocket,
    pub priority: u16,
    pub protobuf: String,
}

impl Input {
    pub fn socket(&self) -> &UdpSocket {
        &self.socket
    }

    pub fn address(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }
    pub fn priority(&self) -> u16 {
        self.priority
    }
    pub fn protobuf(&self) -> &str {
        &self.protobuf
    }
}