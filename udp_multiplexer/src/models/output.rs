use std::io;
use std::net::{SocketAddr, UdpSocket};

pub struct Output {
    pub socket: UdpSocket,
    pub protobuf: String,
}

impl Output {
    pub fn socket(&self) -> &UdpSocket {
        &self.socket
    }
    pub fn address(&self) -> io::Result<SocketAddr> {
        self.socket.peer_addr()
    }
    pub fn protobuf(&self) -> &str {
        &self.protobuf
    }
}