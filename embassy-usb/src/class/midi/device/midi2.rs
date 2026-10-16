//! USB MIDI 2.0 device with a MIDI 1.0-compatible alternate setting.

use super::{
    CS_ENDPOINT, CS_INTERFACE, MS_HEADER_SUBTYPE, MidiClass, MidiClassConfig, MidiClassState, MidiEndpoints,
    PROTOCOL_NONE, USB_AUDIO_CLASS, USB_MIDISTREAMING_SUBCLASS,
};
use crate::control::{InResponse, Recipient, Request, RequestType};
use crate::descriptor::{SynchronizationType, UsageType};
use crate::driver::{Driver, EndpointError, EndpointType};
use crate::types::StringIndex;
use crate::{Builder, Handler};

const CS_GR_TRM_BLOCK: u8 = 0x26;

/// Configuration for a MIDI 2.0 interface with MIDI 1.0 compatibility.
///
/// The jack fields configure alternate setting 0; the group fields configure
/// alternate setting 1. Jack directions follow USB MIDI 1.0 terminology.
/// The default is one jack and one group in each direction with 64-byte endpoints.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Midi2ClassConfig<'d> {
    /// Number of jacks for sending data to the host via the IN endpoint.
    /// If set to 0, the IN endpoint will not be allocated on alternate setting 0.
    pub n_in_jacks: u8,
    /// Number of jacks for receiving data from the host via the OUT endpoint.
    /// If set to 0, the OUT endpoint will not be allocated on alternate setting 0.
    pub n_out_jacks: u8,
    /// Maximum packet size for both alternate settings.
    /// For full-speed devices, the value has to be one of 8, 16, 32 or 64.
    pub max_packet_size: u16,
    /// Name of the MIDIStreaming interface.
    pub interface_name: Option<&'d str>,
    /// Name of each MIDI 1.0 IN jack.
    pub in_jack_names: &'d [Option<&'d str>],
    /// Name of each MIDI 1.0 OUT jack.
    pub out_jack_names: &'d [Option<&'d str>],
    /// Number of UMP groups received from the host on alternate setting 1.
    pub in_groups: u8,
    /// Number of UMP groups sent to the host on alternate setting 1.
    pub out_groups: u8,
}

impl Default for Midi2ClassConfig<'_> {
    fn default() -> Self {
        Self {
            n_in_jacks: 1,
            n_out_jacks: 1,
            max_packet_size: 64,
            interface_name: None,
            in_jack_names: &[],
            out_jack_names: &[],
            in_groups: 1,
            out_groups: 1,
        }
    }
}

/// State for the MIDI 1.0 names and MIDI 2.0 Group Terminal Block descriptors.
pub struct Midi2ClassState<'d> {
    names: MidiClassState<'d>,
    interface: u8,
    descriptors: [u8; 31],
    len: usize,
}

impl Midi2ClassState<'_> {
    /// Creates unconfigured state for [`Midi2Class::new`].
    pub const fn new() -> Self {
        Self {
            names: MidiClassState::new(),
            interface: 0,
            descriptors: [0; 31],
            len: 0,
        }
    }

    fn configure(&mut self, interface: u8, in_groups: u8, out_groups: u8) {
        self.interface = interface;
        let count = usize::from(in_groups != 0) + usize::from(out_groups != 0);
        self.len = 5 + count * 13;
        self.descriptors[..5].copy_from_slice(&[5, CS_GR_TRM_BLOCK, 1, self.len as u8, (self.len >> 8) as u8]);

        let mut pos = 5;
        if in_groups != 0 {
            self.descriptors[pos..pos + 13].copy_from_slice(&[
                13,
                CS_GR_TRM_BLOCK,
                2,
                1,
                1,
                0,
                in_groups,
                0,
                0x11,
                0,
                0,
                0,
                0,
            ]);
            pos += 13;
        }
        if out_groups != 0 {
            self.descriptors[pos..pos + 13].copy_from_slice(&[
                13,
                CS_GR_TRM_BLOCK,
                2,
                2,
                2,
                0,
                out_groups,
                0,
                0x11,
                0,
                0,
                0,
                0,
            ]);
        }
    }
}

impl Default for Midi2ClassState<'_> {
    fn default() -> Self {
        Self::new()
    }
}

impl Handler for Midi2ClassState<'_> {
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type == RequestType::Standard
            && req.recipient == Recipient::Interface
            && req.request == Request::GET_DESCRIPTOR
            && req.descriptor_type_index() == (CS_GR_TRM_BLOCK, 1)
            && req.index == u16::from(self.interface)
        {
            Some(InResponse::Accepted(&self.descriptors[..self.len]))
        } else {
            None
        }
    }

    fn get_string(&mut self, index: StringIndex, lang_id: u16) -> Option<&str> {
        self.names.get_string(index, lang_id)
    }
}

/// Packet-level implementation of a USB MIDI 2.0 device.
///
/// MIDI 1.0 is available on alternate setting 0 and UMP on setting 1.
/// `read_packet` requires a buffer at least as large as `max_packet_size`, and
/// `write_packet` must not exceed that size. The caller supplies complete UMP
/// messages and handles short packets or a ZLP when ending a bulk transfer.
pub struct Midi2Class<'d, D: Driver<'d>> {
    midi1: MidiClass<'d, D>,
    endpoints: MidiEndpoints<'d, D>,
}

impl<'d, D: Driver<'d>> Midi2Class<'d, D> {
    /// Creates a MIDI class with both alternate settings and named MIDI 1.0 jacks.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut Midi2ClassState<'d>, config: Midi2ClassConfig<'d>) -> Self {
        let Midi2ClassConfig {
            n_in_jacks,
            n_out_jacks,
            max_packet_size,
            interface_name,
            in_jack_names,
            out_jack_names,
            in_groups,
            out_groups,
        } = config;
        assert!(in_groups != 0 || out_groups != 0, "MIDI 2.0 needs at least one group");
        assert!(
            in_groups <= 16 && out_groups <= 16,
            "MIDI 2.0 supports at most 16 groups per direction"
        );

        let midi1 = MidiClassConfig {
            n_in_jacks,
            n_out_jacks,
            max_packet_size,
            interface_name,
            in_jack_names,
            out_jack_names,
        };
        state.names.configure(builder, &midi1);
        let interface_name = state.names.interface();
        let (midi1, (interface, read_ep, write_ep)) =
            MidiClass::build_with_alt(builder, midi1, &state.names, |iface| {
                let interface = u8::from(iface.interface_number());
                let mut alt = iface.alt_setting(
                    USB_AUDIO_CLASS,
                    USB_MIDISTREAMING_SUBCLASS,
                    PROTOCOL_NONE,
                    interface_name,
                );
                // MIDI 2.0 has only the MS header in its class-specific interface descriptors.
                alt.descriptor(CS_INTERFACE, &[MS_HEADER_SUBTYPE, 0x00, 0x02, 7, 0]);

                let read_ep = if in_groups != 0 {
                    let ep = alt.endpoint_out(
                        EndpointType::Bulk,
                        None,
                        max_packet_size,
                        0,
                        SynchronizationType::NoSynchronization,
                        UsageType::DataEndpoint,
                        &[],
                    );
                    alt.descriptor(CS_ENDPOINT, &[2, 1, 1]);
                    Some(ep)
                } else {
                    None
                };

                let write_ep = if out_groups != 0 {
                    let ep = alt.endpoint_in(
                        EndpointType::Bulk,
                        None,
                        max_packet_size,
                        0,
                        SynchronizationType::NoSynchronization,
                        UsageType::DataEndpoint,
                        &[],
                    );
                    alt.descriptor(CS_ENDPOINT, &[2, 1, 2]);
                    Some(ep)
                } else {
                    None
                };
                (interface, read_ep, write_ep)
            });

        state.configure(interface, in_groups, out_groups);
        builder.handler(state);
        Self {
            midi1,
            endpoints: MidiEndpoints::new(read_ep, write_ep),
        }
    }

    /// Accesses the MIDI 1.0 packet transport, active on alternate setting 0.
    pub fn midi1(&mut self) -> &mut MidiClass<'d, D> {
        &mut self.midi1
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        self.endpoints.max_packet_size()
    }

    /// Writes one USB transfer containing complete UMP packets on alternate setting 1.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.endpoints.write_packet(data).await
    }

    /// Reads one USB transfer containing UMP data on alternate setting 1.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.endpoints.read_packet(data).await
    }

    /// Waits until the host selects the MIDI 2.0 alternate setting.
    pub async fn wait_connection(&mut self) {
        self.endpoints.wait_connection().await;
    }
}
