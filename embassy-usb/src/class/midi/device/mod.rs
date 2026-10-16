//! USB MIDI device classes.

pub mod midi1;
pub mod midi2;

pub use midi1::*;

use super::{MS_HEADER_SUBTYPE, PROTOCOL_NONE, USB_AUDIO_CLASS, USB_MIDISTREAMING_SUBCLASS};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};

const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

struct MidiEndpoints<'d, D: Driver<'d>> {
    read_ep: Option<D::EndpointOut>,
    write_ep: Option<D::EndpointIn>,
}

impl<'d, D: Driver<'d>> MidiEndpoints<'d, D> {
    fn new(read_ep: Option<D::EndpointOut>, write_ep: Option<D::EndpointIn>) -> Self {
        Self { read_ep, write_ep }
    }

    fn max_packet_size(&self) -> u16 {
        if let Some(read_ep) = &self.read_ep {
            read_ep.info().max_packet_size
        } else if let Some(write_ep) = &self.write_ep {
            write_ep.info().max_packet_size
        } else {
            0
        }
    }

    async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.as_mut().ok_or(EndpointError::Disabled)?.read(data).await
    }

    async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.as_mut().ok_or(EndpointError::Disabled)?.write(data).await
    }

    async fn wait_read_connection(&mut self) {
        if let Some(read_ep) = &mut self.read_ep {
            read_ep.wait_enabled().await;
        }
    }

    async fn wait_connection(&mut self) {
        if self.read_ep.is_some() {
            self.wait_read_connection().await;
        } else if let Some(write_ep) = &mut self.write_ep {
            write_ep.wait_enabled().await;
        }
    }
}
