use embassy_futures::block_on;

use super::*;
use crate::driver::{Endpoint, EndpointIn, EndpointOut};
use crate::mocks::device::{Response, TestDriver, with_builder, with_device};

fn request(
    usb: &mut UsbDevice<'_, TestDriver>,
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
) -> Response {
    let value = value.to_le_bytes();
    let index = index.to_le_bytes();
    let length = length.to_le_bytes();
    block_on(usb.handle_control([
        request_type,
        request,
        value[0],
        value[1],
        index[0],
        index[1],
        length[0],
        length[1],
    ]));
    usb.control.response.take().unwrap()
}

fn endpoint_request(usb: &mut UsbDevice<'_, TestDriver>, code: u8, index: u16) -> Response {
    if code == Request::GET_STATUS {
        request(usb, 0x82, code, 0, index, 2)
    } else {
        request(usb, 0x02, code, 0, index, 0)
    }
}

fn assert_rejected(usb: &mut UsbDevice<'_, TestDriver>, code: u8, index: u16) {
    let calls = usb.inner.bus.halt_calls;
    assert_eq!(endpoint_request(usb, code, index), Response::Rejected);
    assert_eq!(usb.inner.bus.halt_calls, calls);
}

fn configure(usb: &mut UsbDevice<'_, TestDriver>) {
    block_on(usb.inner.handle_bus_event(Event::PowerDetected));
    block_on(usb.inner.handle_bus_event(Event::Reset));
    assert_eq!(request(usb, 0, Request::SET_ADDRESS, 1, 0, 0), Response::Accepted);
    assert_eq!(request(usb, 0, Request::SET_CONFIGURATION, 1, 0, 0), Response::Accepted);
}

#[test]
fn test_get_descriptors() {
    with_device(|usb| {
        block_on(usb.inner.handle_bus_event(Event::PowerDetected));
        block_on(usb.inner.handle_bus_event(Event::Reset));

        let Response::Data(device) = request(usb, 0x80, Request::GET_DESCRIPTOR, 0x0100, 0, 18) else {
            panic!("device descriptor was rejected");
        };
        assert_eq!(device.len(), 18);
        assert_eq!(u16::from_le_bytes([device[8], device[9]]), 0x1234);
        assert_eq!(u16::from_le_bytes([device[10], device[11]]), 0x5678);

        let Response::Data(config) = request(usb, 0x80, Request::GET_DESCRIPTOR, 0x0200, 0, 256) else {
            panic!("configuration descriptor was rejected");
        };
        // Longer than one EP0 packet, so the reply is split across data stages.
        const EP0_MAX_PACKET_SIZE: usize = 64;
        assert!(config.len() > EP0_MAX_PACKET_SIZE);
        assert_eq!(config.len(), u16::from_le_bytes([config[2], config[3]]) as usize);
    });
}

#[test]
fn test_endpoint_packets() {
    let mut driver = TestDriver::default();
    let out_handle = driver.endpoint(0x01.into());
    let in_handle = driver.endpoint(0x82.into());
    driver
        .alloc_endpoint_in(EndpointType::Bulk, Some(0x81.into()), 4, 0)
        .unwrap();
    let mut out_ep = driver.alloc_endpoint_out(EndpointType::Bulk, None, 4, 0).unwrap();
    let mut in_ep = driver.alloc_endpoint_in(EndpointType::Bulk, None, 4, 0).unwrap();
    assert_eq!(out_ep.info().addr, 0x01.into());
    assert_eq!(in_ep.info().addr, 0x82.into());

    out_handle.queue_read(&[1, 2, 3, 4]);
    out_handle.queue_read(&[5, 6]);
    let mut buf = [0; 8];
    let n = block_on(out_ep.read_transfer(&mut buf)).unwrap();
    assert_eq!(&buf[..n], &[1, 2, 3, 4, 5, 6]);

    block_on(in_ep.write_transfer(&[7, 8, 9, 10], true)).unwrap();
    let writes = in_handle.take_writes();
    assert_eq!(writes.len(), 2);
    assert_eq!(writes[0], [7, 8, 9, 10]);
    assert!(writes[1].is_empty());
}

#[test]
fn test_builder_assigns_endpoint_addresses() {
    use crate::class::msc::{USB_CLASS_MSC, USB_PROTOCOL_BULK_ONLY, USB_SUBCLASS_SCSI_TRANSPARENT};

    with_builder(
        |builder| {
            let mut function = builder.function(USB_CLASS_MSC, USB_SUBCLASS_SCSI_TRANSPARENT, USB_PROTOCOL_BULK_ONLY);
            let mut interface = function.interface();
            let mut alt = interface.alt_setting(
                USB_CLASS_MSC,
                USB_SUBCLASS_SCSI_TRANSPARENT,
                USB_PROTOCOL_BULK_ONLY,
                None,
            );
            alt.endpoint_bulk_out(None, 64);
            alt.endpoint_bulk_in(None, 64);
        },
        |usb| {
            block_on(usb.inner.handle_bus_event(Event::PowerDetected));
            block_on(usb.inner.handle_bus_event(Event::Reset));
            let Response::Data(config) = request(usb, 0x80, Request::GET_DESCRIPTOR, 0x0200, 0, 256) else {
                panic!("configuration descriptor was rejected");
            };
            let info = crate::class::msc::host::find_msc(&config).unwrap();
            assert_eq!(info.bulk_out_ep, 0x01);
            assert_eq!(info.bulk_in_ep, 0x81);
        },
    );
}

#[test]
fn test_handle_control_endpoint_out_of_range() {
    with_device(|usb| {
        configure(usb);
        // Endpoint 9 is outside the controller's register array.
        block_on(usb.handle_control([0x82, 0, 0, 0, 0x89, 0, 2, 0]));
        assert_eq!(usb.control.response.take(), Some(Response::Rejected));
        assert_eq!(usb.inner.bus.halt_calls, 0);
    });
}

#[test]
fn test_handle_control_endpoint_index() {
    with_device(|usb| {
        configure(usb);
        for index in 0..=u16::MAX {
            // Only these endpoints exist in the default alternate setting.
            if matches!(index, 0 | 0x80 | 1 | 0x82) {
                continue;
            }
            for code in [Request::GET_STATUS, Request::SET_FEATURE, Request::CLEAR_FEATURE] {
                assert_rejected(usb, code, index);
            }
        }
    });
}

#[test]
fn test_handle_control_endpoint_halt() {
    with_device(|usb| {
        configure(usb);
        for index in [1, 0x82] {
            assert_eq!(endpoint_request(usb, Request::GET_STATUS, index), Response::status(0));
            assert_eq!(endpoint_request(usb, Request::SET_FEATURE, index), Response::Accepted);
            assert_eq!(endpoint_request(usb, Request::GET_STATUS, index), Response::status(1));
            assert_eq!(endpoint_request(usb, Request::CLEAR_FEATURE, index), Response::Accepted);
            assert_eq!(endpoint_request(usb, Request::GET_STATUS, index), Response::status(0));
        }
        assert_eq!(usb.inner.bus.halt_calls, 10);
    });
}

#[test]
fn test_handle_control_endpoint_state() {
    with_device(|usb| {
        let assert_unconfigured = |usb: &mut UsbDevice<'_, TestDriver>| {
            for index in [1, 0x82, 0x83, 4] {
                for code in [Request::GET_STATUS, Request::SET_FEATURE, Request::CLEAR_FEATURE] {
                    assert_rejected(usb, code, index);
                }
            }
        };
        assert_unconfigured(usb);
        block_on(usb.inner.handle_bus_event(Event::PowerDetected));
        assert_unconfigured(usb);
        assert_eq!(request(usb, 0, Request::SET_ADDRESS, 1, 0, 0), Response::Accepted);
        assert_unconfigured(usb);
        assert_eq!(request(usb, 0, Request::SET_CONFIGURATION, 1, 0, 0), Response::Accepted);
        assert_eq!(endpoint_request(usb, Request::GET_STATUS, 1), Response::status(0));
        assert_eq!(request(usb, 0, Request::SET_CONFIGURATION, 0, 0, 0), Response::Accepted);
        assert_unconfigured(usb);
        configure(usb);
        block_on(usb.inner.handle_bus_event(Event::Reset));
        assert_unconfigured(usb);
        configure(usb);
        block_on(usb.inner.handle_bus_event(Event::PowerRemoved));
        assert_unconfigured(usb);
        configure(usb);
        block_on(usb.disable());
        assert_unconfigured(usb);
    });
}

#[test]
fn test_handle_control_endpoint_alt_setting() {
    with_device(|usb| {
        configure(usb);
        assert_eq!(request(usb, 1, Request::SET_INTERFACE, 1, 0, 0), Response::Accepted);
        for code in [Request::GET_STATUS, Request::SET_FEATURE, Request::CLEAR_FEATURE] {
            for index in [1, 0x82, 3, 0x84] {
                assert_rejected(usb, code, index);
            }
        }
        assert_eq!(endpoint_request(usb, Request::SET_FEATURE, 0x83), Response::Accepted);
        assert_eq!(endpoint_request(usb, Request::GET_STATUS, 0x83), Response::status(1));
        assert_eq!(endpoint_request(usb, Request::CLEAR_FEATURE, 0x83), Response::Accepted);
        assert_eq!(endpoint_request(usb, Request::GET_STATUS, 0x83), Response::status(0));

        // Isochronous endpoints exist, but have no halt feature.
        let calls = usb.inner.bus.halt_calls;
        assert_eq!(endpoint_request(usb, Request::GET_STATUS, 4), Response::status(0));
        assert_rejected(usb, Request::SET_FEATURE, 4);
        assert_rejected(usb, Request::CLEAR_FEATURE, 4);
        assert_eq!(usb.inner.bus.halt_calls, calls);

        assert_eq!(request(usb, 1, Request::SET_INTERFACE, 0, 0, 0), Response::Accepted);
        for code in [Request::GET_STATUS, Request::SET_FEATURE, Request::CLEAR_FEATURE] {
            assert_rejected(usb, code, 0x83);
            assert_rejected(usb, code, 4);
        }
        assert_eq!(endpoint_request(usb, Request::GET_STATUS, 1), Response::status(0));
    });
}

#[test]
fn test_handle_control_endpoint_zero() {
    with_device(|usb| {
        block_on(usb.inner.handle_bus_event(Event::PowerDetected));
        for index in [0, 0x80] {
            assert_rejected(usb, Request::GET_STATUS, index);
        }
        assert_eq!(request(usb, 0, Request::SET_ADDRESS, 1, 0, 0), Response::Accepted);
        for configuration in [0, 1] {
            assert_eq!(
                request(usb, 0, Request::SET_CONFIGURATION, configuration, 0, 0),
                Response::Accepted
            );
            for index in [0, 0x80] {
                assert_eq!(endpoint_request(usb, Request::GET_STATUS, index), Response::status(0));
                assert_rejected(usb, Request::SET_FEATURE, index);
                assert_rejected(usb, Request::CLEAR_FEATURE, index);
            }
        }
        assert_eq!(usb.inner.bus.halt_calls, 0);
    });
}

#[test]
fn test_handle_control_endpoint_request_fields() {
    with_device(|usb| {
        configure(usb);
        for (value, length) in [(1, 2), (0, 0), (0, 1), (0, 3)] {
            assert_eq!(
                request(usb, 0x82, Request::GET_STATUS, value, 1, length),
                Response::Rejected
            );
        }
        for code in [Request::SET_FEATURE, Request::CLEAR_FEATURE] {
            for (value, length) in [(1, 0), (0, 1)] {
                assert_eq!(request(usb, 0x02, code, value, 1, length), Response::Rejected);
            }
        }
        assert_eq!(usb.inner.bus.halt_calls, 0);
    });
}
