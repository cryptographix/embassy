use core::cell::Cell;
use core::marker::PhantomData;

use crate::driver::host::{
    AddressSet, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::driver::{EndpointInfo, EndpointType};
use crate::host::control::SetupPacket;

/// Allocator that records the set passed to `device_removed`.
#[derive(Clone)]
pub(crate) struct Recorder<'a>(pub &'a Cell<AddressSet>, pub Option<&'a dyn Fn(AddressSet)>);

pub(crate) enum NoPipe {}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for NoPipe {
    async fn control_in(&mut self, _: &[u8; 8], _: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        match *self {}
    }

    async fn control_out(&mut self, _: &[u8; 8], _: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        match *self {}
    }

    async fn request_in(&mut self, _: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        match *self {}
    }

    async fn request_out(&mut self, _: &[u8], _: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        match *self {}
    }

    fn set_timeout(&mut self, _: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        match *self {}
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        match *self {}
    }
}

impl<'a> UsbHostAllocator<'a> for Recorder<'a> {
    type Pipe<T: pipe::Type, D: pipe::Direction> = NoPipe;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        _: u8,
        _: &EndpointInfo,
        _: Option<SplitInfo>,
    ) -> Result<NoPipe, HostError> {
        Err(HostError::OutOfPipes)
    }

    fn device_removed(&self, addrs: AddressSet) {
        self.0.set(addrs);
        if let Some(on_removed) = self.1 {
            on_removed(addrs);
        }
    }
}

/// Expected calls made by a control-only host test.
#[derive(Clone, Copy)]
pub(crate) enum ControlStep<'a> {
    Allocate {
        address: u8,
        max_packet_size: u16,
    },
    In {
        setup: SetupPacket,
        data: &'a [u8],
    },
    InError {
        setup: SetupPacket,
        error: PipeError,
    },
    Out {
        setup: SetupPacket,
        data: &'a [u8],
    },
    OutError {
        setup: SetupPacket,
        data: &'a [u8],
        error: PipeError,
    },
}

pub(crate) struct ControlScript<'a> {
    pub steps: &'a [ControlStep<'a>],
    pub next: Cell<usize>,
}

impl<'a> ControlScript<'a> {
    pub fn new(steps: &'a [ControlStep<'a>]) -> Self {
        Self {
            steps,
            next: Cell::new(0),
        }
    }

    pub fn allocator(&'a self) -> ScriptedAllocator<'a> {
        ScriptedAllocator(self)
    }

    pub fn take(&self) -> ControlStep<'a> {
        let next = self.next.get();
        self.next.set(next + 1);
        *self.steps.get(next).expect("unexpected control pipe call")
    }

    pub fn assert_finished(&self) {
        assert_eq!(self.next.get(), self.steps.len(), "unused control pipe steps");
    }
}

#[derive(Clone)]
pub(crate) struct ScriptedAllocator<'a>(pub &'a ControlScript<'a>);

pub(crate) struct ScriptedPipe<'a, T, D> {
    pub script: &'a ControlScript<'a>,
    pub _marker: PhantomData<(T, D)>,
}

impl<'d, 'a: 'd> UsbHostAllocator<'d> for ScriptedAllocator<'a> {
    type Pipe<T: pipe::Type, D: pipe::Direction> = ScriptedPipe<'a, T, D>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        assert_eq!(T::ep_type(), EndpointType::Control);
        assert_eq!(endpoint.ep_type, EndpointType::Control);
        assert_eq!(endpoint.addr.index(), 0);
        assert!(endpoint.addr.is_in());
        assert_eq!(split, None);
        match self.0.take() {
            ControlStep::Allocate {
                address,
                max_packet_size,
            } => {
                assert_eq!(addr, address);
                assert_eq!(endpoint.max_packet_size, max_packet_size);
            }
            _ => panic!("expected control pipe allocation"),
        }
        Ok(ScriptedPipe {
            script: self.0,
            _marker: PhantomData,
        })
    }
}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for ScriptedPipe<'_, T, D> {
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        match self.script.take() {
            ControlStep::In { setup: expected, data } => {
                assert_eq!(SetupPacket::from_bytes(*setup), expected);
                assert!(data.len() <= buf.len());
                buf[..data.len()].copy_from_slice(data);
                Ok(data.len())
            }
            ControlStep::InError { setup: expected, error } => {
                assert_eq!(SetupPacket::from_bytes(*setup), expected);
                Err(error)
            }
            _ => panic!("expected control IN transfer"),
        }
    }

    async fn control_out(&mut self, setup: &[u8; 8], data: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        match self.script.take() {
            ControlStep::Out {
                setup: expected,
                data: expected_data,
            } => {
                assert_eq!(SetupPacket::from_bytes(*setup), expected);
                assert_eq!(data, expected_data);
                Ok(())
            }
            ControlStep::OutError {
                setup: expected,
                data: expected_data,
                error,
            } => {
                assert_eq!(SetupPacket::from_bytes(*setup), expected);
                assert_eq!(data, expected_data);
                Err(error)
            }
            _ => panic!("expected control OUT transfer"),
        }
    }

    async fn request_in(&mut self, _: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        panic!("bulk and interrupt transfers are not scripted")
    }

    async fn request_out(&mut self, _: &[u8], _: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        panic!("bulk and interrupt transfers are not scripted")
    }

    fn set_timeout(&mut self, _: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        panic!("bulk and interrupt transfers are not scripted")
    }
}
