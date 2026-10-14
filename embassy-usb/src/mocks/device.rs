use core::future::pending;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;

use crate::descriptor::{SynchronizationType, UsageType};
use crate::device::{Builder, Config, UsbDevice};
use crate::driver::{
    Bus, ControlPipe, Direction, Driver, Endpoint, EndpointAddress, EndpointAllocError, EndpointError, EndpointIn,
    EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
};

pub(crate) struct TestDriver {
    pub bus: TestBus,
    pub endpoints: [Rc<RefCell<EndpointPackets>>; 32],
}

#[derive(Default)]
pub(crate) struct EndpointPackets {
    pub reads: VecDeque<Vec<u8>>,
    pub writes: Vec<Vec<u8>>,
}

pub(crate) struct TestEndpoint {
    pub info: EndpointInfo,
    pub packets: Rc<RefCell<EndpointPackets>>,
}

pub(crate) struct EndpointHandle(pub Rc<RefCell<EndpointPackets>>);

impl EndpointHandle {
    pub fn queue_read(&self, packet: &[u8]) {
        self.0.borrow_mut().reads.push_back(packet.to_vec());
    }

    pub fn take_writes(&self) -> Vec<Vec<u8>> {
        core::mem::take(&mut self.0.borrow_mut().writes)
    }
}

impl Default for TestDriver {
    fn default() -> Self {
        Self {
            bus: TestBus::default(),
            endpoints: core::array::from_fn(|_| Rc::new(RefCell::new(EndpointPackets::default()))),
        }
    }
}

impl TestDriver {
    pub fn endpoint(&self, addr: EndpointAddress) -> EndpointHandle {
        EndpointHandle(Rc::clone(&self.endpoints[TestBus::slot(addr)]))
    }

    pub fn allocate(&mut self, info: EndpointInfo) -> Result<TestEndpoint, EndpointAllocError> {
        self.bus.allocate(info)?;
        Ok(TestEndpoint {
            info,
            packets: Rc::clone(&self.endpoints[TestBus::slot(info.addr)]),
        })
    }
}

impl<'d> Driver<'d> for TestDriver {
    type EndpointOut = TestEndpoint;
    type EndpointIn = TestEndpoint;
    type ControlPipe = TestControlPipe;
    type Bus = TestBus;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<TestEndpoint, EndpointAllocError> {
        let addr = ep_addr
            .or_else(|| self.bus.next_address(Direction::In))
            .ok_or(EndpointAllocError)?;
        assert!(addr.is_in());
        self.allocate(EndpointInfo {
            addr,
            ep_type,
            max_packet_size,
            interval_ms,
        })
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<TestEndpoint, EndpointAllocError> {
        let addr = ep_addr
            .or_else(|| self.bus.next_address(Direction::Out))
            .ok_or(EndpointAllocError)?;
        assert!(addr.is_out());
        self.allocate(EndpointInfo {
            addr,
            ep_type,
            max_packet_size,
            interval_ms,
        })
    }

    fn start(self, _control_max_packet_size: u16) -> (TestBus, TestControlPipe) {
        (self.bus, TestControlPipe::default())
    }
}

impl Endpoint for TestEndpoint {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    // Endpoint traffic is explicitly scripted, so the test endpoint is ready immediately.
    async fn wait_enabled(&mut self) {}
}

impl EndpointIn for TestEndpoint {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        self.packets.borrow_mut().writes.push(buf.to_vec());
        Ok(())
    }
}

impl EndpointOut for TestEndpoint {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        // An empty queue blocks like an idle bus, so tests can end a run by dropping it.
        let packet = loop {
            let queued = self.packets.borrow_mut().reads.pop_front();
            match queued {
                Some(packet) => break packet,
                None => pending::<()>().await,
            }
        };
        if packet.len() > self.info.max_packet_size as usize || packet.len() > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        buf[..packet.len()].copy_from_slice(&packet);
        Ok(packet.len())
    }
}

#[derive(Default)]
pub(crate) struct TestBus {
    pub endpoints: [Option<EndpointInfo>; 32],
    pub enabled: [bool; 32],
    pub stalled: [bool; 32],
    pub halt_calls: usize,
}

impl TestBus {
    pub fn slot(addr: EndpointAddress) -> usize {
        // Model a controller with nine endpoint register pairs, including EP0.
        assert!(addr.index() < 9);
        addr.index() * 2 + usize::from(addr.is_in())
    }

    pub fn next_address(&self, direction: Direction) -> Option<EndpointAddress> {
        (1..9)
            .map(|index| EndpointAddress::from_parts(index, direction))
            .find(|&addr| self.endpoints[Self::slot(addr)].is_none())
    }

    pub fn allocate(&mut self, info: EndpointInfo) -> Result<(), EndpointAllocError> {
        let slot = Self::slot(info.addr);
        assert!(self.endpoints[slot].replace(info).is_none());
        Ok(())
    }

    pub fn check_halt(&mut self, addr: EndpointAddress) -> usize {
        let slot = Self::slot(addr);
        let endpoint = self.endpoints[slot].unwrap();
        assert!(self.enabled[slot]);
        assert!(matches!(endpoint.ep_type, EndpointType::Bulk | EndpointType::Interrupt));
        self.halt_calls += 1;
        slot
    }
}

impl Bus for TestBus {
    async fn enable(&mut self) {}

    async fn disable(&mut self) {
        self.enabled.fill(false);
    }

    async fn poll(&mut self) -> Event {
        pending().await
    }

    fn endpoint_set_enabled(&mut self, addr: EndpointAddress, enabled: bool) {
        let slot = Self::slot(addr);
        assert!(self.endpoints[slot].is_some());
        self.enabled[slot] = enabled;
        self.stalled[slot] = false;
    }

    fn endpoint_set_stalled(&mut self, addr: EndpointAddress, stalled: bool) {
        let slot = self.check_halt(addr);
        self.stalled[slot] = stalled;
    }

    fn endpoint_is_stalled(&mut self, addr: EndpointAddress) -> bool {
        let slot = self.check_halt(addr);
        self.stalled[slot]
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Response {
    Accepted,
    Data(Vec<u8>),
    Rejected,
}

impl Response {
    pub fn status(value: u16) -> Self {
        Self::Data(value.to_le_bytes().into())
    }
}

#[derive(Default)]
pub(crate) struct TestControlPipe {
    pub response: Option<Response>,
}

impl ControlPipe for TestControlPipe {
    fn max_packet_size(&self) -> usize {
        64
    }

    async fn setup(&mut self) -> [u8; 8] {
        unreachable!()
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        buf.fill(0);
        Ok(buf.len())
    }

    async fn data_in(&mut self, data: &[u8], first: bool, _last: bool) -> Result<(), EndpointError> {
        if first {
            self.response = Some(Response::Data(Vec::new()));
        }
        match self.response.as_mut() {
            Some(Response::Data(bytes)) => bytes.extend_from_slice(data),
            _ => panic!("control IN data must start with the first packet"),
        }
        Ok(())
    }

    async fn accept(&mut self) {
        self.response = Some(Response::Accepted);
    }

    async fn reject(&mut self) {
        self.response = Some(Response::Rejected);
    }

    async fn accept_set_address(&mut self, _addr: u8) {
        self.accept().await;
    }
}

pub(crate) fn with_builder(
    configure: impl FnOnce(&mut Builder<'_, TestDriver>),
    test: impl FnOnce(&mut UsbDevice<'_, TestDriver>),
) {
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut control_buf = [0; 64];
    let mut builder = Builder::new(
        TestDriver::default(),
        Config::new(0x1234, 0x5678),
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    configure(&mut builder);
    test(&mut builder.build());
}

pub(crate) fn with_device(test: impl FnOnce(&mut UsbDevice<'_, TestDriver>)) {
    with_builder(
        |builder| {
            let mut function = builder.function(0xff, 0, 0);
            let mut interface = function.interface();
            let mut alt = interface.alt_setting(0xff, 0, 0, None);
            alt.endpoint_bulk_out(Some(0x01.into()), 64);
            alt.endpoint_bulk_in(Some(0x82.into()), 64);
            // Make configuration descriptor reads span more than one EP0 packet.
            alt.descriptor(0x24, &[0; 24]);
            let mut alt = interface.alt_setting(0xff, 0, 0, None);
            alt.endpoint_interrupt_in(Some(0x83.into()), 64, 1);
            alt.endpoint_isochronous_out(
                Some(0x04.into()),
                64,
                1,
                SynchronizationType::Asynchronous,
                UsageType::DataEndpoint,
                &[],
            );
            drop(function);
        },
        test,
    );
}
