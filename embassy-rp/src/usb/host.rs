use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, SplitSpeed, TimeoutConfig, UsbHostAllocator, UsbHostController,
    UsbPipe, pipe,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};

/// Reduce a [`SplitInfo`] to the legacy "emit PRE packet" bit used by this
/// full-speed only controller. USB 1.1 §11.8.6: PRE is required when the
/// target device is low-speed and reached through a (full-speed) hub.
fn split_to_pre(split: Option<SplitInfo>) -> bool {
    matches!(split, Some(s) if s.device_speed() == SplitSpeed::Low)
}
use rp_pac::usb_dpram::vals::EpControlEndpointType;

use super::{BUS_WAKER, DPRAM_DATA_OFFSET, EP_COUNT, EP_IN_WAKERS, EP_MEMORY, EndpointBuffer, Instance};
#[path = "host_epx.rs"]
mod epx;

use epx::*;

use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::interrupt::{self};
use crate::peripherals::USB;
use crate::usb::EP_MEMORY_SIZE;
use crate::{Peri, RegExt};

/// Root port reset drive time (USB 2.0 §7.1.7.5, TDRSTR).
const ROOT_RESET_MS: u64 = 50;
/// Reset recovery time (USB 2.0 §7.1.7.5, TRSTRCY). A device need not answer any request
/// until this has elapsed after reset is deasserted.
const RESET_RECOVERY_MS: u64 = 10;
/// Register writes need two clk_usb cycles to transfer; 12 clk_sys cycles cover this at the supported 150 MHz maximum.
const USB_CLOCK_DELAY_CYCLES: u32 = 12;
const SIE_START_DELAY_CYCLES: u32 = 12;

/// Default control timeout, overriding the trait's 50 ms: devices may overrun the
/// USB 2.0 §9.2.6.4 response requirement while configuring.
#[allow(clippy::field_reassign_with_default)]
fn default_timeout() -> TimeoutConfig {
    let mut timeout = TimeoutConfig::default();
    timeout.no_data_timeout = core::time::Duration::from_millis(500);
    timeout
}

/// State of the dedicated interrupt endpoints, which the controller polls on its own.
///
/// Each pipe owns its endpoint outright, so nothing here arbitrates.
struct PolledState {
    /// Bitset of allocated interrupt pipes.
    allocated_pipes: AtomicU16,
}

/// Per-instance state shared between [`Driver`], [`Allocator`] and [`Channel`].
pub struct HostState {
    epx: EpxState,
    polled: PolledState,
}

impl HostState {
    /// Create a new, reset host state.
    pub const fn new() -> Self {
        Self {
            epx: EpxState::new(),
            polled: PolledState {
                allocated_pipes: AtomicU16::new(0),
            },
        }
    }

    fn reset(&self) {
        self.epx.reset();
        self.polled.allocated_pipes.store(0, Ordering::Relaxed);
    }
}

/// Sealed extension of [`Instance`] exposing the per-peripheral [`HostState`].
#[allow(private_bounds)]
pub trait SealedHostInstance: Instance {
    #[doc(hidden)]
    fn host_state() -> &'static HostState;
}

impl SealedHostInstance for crate::peripherals::USB {
    fn host_state() -> &'static HostState {
        static STATE: HostState = HostState::new();
        &STATE
    }
}

/// RP2040 USB host driver handle.
pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    connected: bool,
}

impl<'d, T: SealedHostInstance> Driver<'d, T> {
    /// Create a new USB driver.
    pub fn new(_usb: Peri<'d, USB>, _irq: impl Binding<T::Interrupt, InterruptHandler<T>>) -> Self {
        let regs = T::regs();

        // Reset the peripheral: zeroing its registers by hand would clobber those that
        // do not reset to zero, notably `USBPHY_TRIM` and `NAK_POLL`.
        crate::pac::RESETS.reset().modify(|w| w.set_usbctrl(true));
        crate::pac::RESETS.reset().modify(|w| w.set_usbctrl(false));
        while !crate::pac::RESETS.reset_done().read().usbctrl() {}

        // DPRAM is outside the peripheral reset, so clear the control area by hand.
        unsafe {
            let p = EP_MEMORY as *mut u32;
            for i in 0..DPRAM_DATA_OFFSET as usize / 4 {
                p.add(i).write_volatile(0)
            }
        }

        regs.usb_muxing().modify(|w| {
            w.set_to_phy(true);
            w.set_softcon(true);
        });
        regs.usb_pwr().modify(|w| {
            w.set_vbus_detect(true);
            w.set_vbus_detect_override_en(true);
        });
        regs.main_ctrl().modify(|w| {
            w.set_controller_en(true);
            w.set_host_ndevice(true);
            // RP2350 resets PHY isolation asserted; the PHY is unusable until cleared (§12.7.2).
            #[cfg(feature = "_rp235x")]
            w.set_phy_iso(false);
        });
        regs.sie_ctrl().modify(|w| {
            w.set_sof_en(true);
            w.set_keep_alive_en(true);
            w.set_pulldown_en(true);
        });

        regs.inte().write(|w| {
            w.set_buff_status(true);
            w.set_host_resume(true);
            w.set_error_data_seq(true);
            w.set_error_crc(true);
            w.set_error_bit_stuff(true);
        });

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // Initialize the bus so that it signals that power is available
        BUS_WAKER.wake();

        // Reset per-instance allocator state.
        T::host_state().reset();

        Self {
            phantom: PhantomData,
            connected: false,
        }
    }
}

/// USB endpoint.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel<'d, T: SealedHostInstance, E, D> {
    _phantom: PhantomData<(&'d mut T, E, D)>,
    index: usize,
    buf: EndpointBuffer<T>,
    dev_addr: u8,

    max_packet_size: u16,
    ep_addr: u8,

    /// Interrupt endpoint poll interval
    interval: u8,

    /// DATA0-DATA1 state
    pid: bool,
    /// Send PRE packet
    pre: bool,
    timeout: TimeoutConfig,
    /// Per-transaction control response budget, or `None` outside a control transfer.
    control_timeout_us: Option<u64>,
}

impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> Channel<'d, T, E, D> {
    fn start_control_timeout(&mut self, has_data: bool) {
        let timeout = if has_data {
            self.timeout.data_timeout
        } else {
            self.timeout.no_data_timeout
        };
        self.control_timeout_us = Some(timeout.as_micros().min(u64::MAX as u128) as u64);
    }

    /// [EP_MEMORY]-relative address
    fn new(index: usize, buf_addr: u16, buf_len: u16, ep_info: &EndpointInfo, dev_addr: u8, pre: bool) -> Self {
        // TODO: assert only in debug?
        assert!(ep_info.ep_type == E::ep_type());
        assert!(buf_addr + buf_len <= EP_MEMORY_SIZE as u16);
        assert!(ep_info.max_packet_size <= buf_len);

        // TODO: Support interrupt OUT
        assert!(!(E::ep_type() == EndpointType::Interrupt && D::is_out()));

        if ep_info.ep_type == EndpointType::Interrupt {
            assert!(index > 0 && index < EP_COUNT);
        } else {
            assert!(index >= EP_COUNT && index < EP_COUNT + EPX_MAX_PIPES);
        }

        Self {
            _phantom: PhantomData,
            index,
            dev_addr,
            buf: EndpointBuffer {
                addr: buf_addr,
                len: buf_len,
                _phantom: PhantomData,
            },
            max_packet_size: ep_info.max_packet_size,
            ep_addr: ep_info.addr.into(),
            interval: ep_info.interval_ms,
            pid: false,
            pre,
            timeout: default_timeout(),
            control_timeout_us: None,
        }
    }
}

pub(super) type BufferControlReg = rp_pac::common::Reg<rp_pac::usb_dpram::regs::EpBufferControl, rp_pac::common::RW>;
pub(super) type EpControlReg = rp_pac::common::Reg<rp_pac::usb_dpram::regs::EpControl, rp_pac::common::RW>;
type AddrControlReg = rp_pac::common::Reg<rp_pac::usb::regs::AddrEndpX, rp_pac::common::RW>;

impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> Channel<'d, T, E, D> {
    /// Get channel waker
    fn waker(&self) -> &AtomicWaker {
        if Self::is_interrupt() {
            &EP_IN_WAKERS[self.index]
        } else {
            &T::host_state().epx.epx_wakers[self.epx_slot()]
        }
    }

    /// Whether this pipe was made to yield EPX since the last check.
    fn take_nak_yield(&self) -> bool {
        T::host_state().epx.take_epx_yielded(self.epx_slot())
    }

    /// Get buffer control register
    fn buffer_control(&self) -> BufferControlReg {
        let index = if Self::is_interrupt() {
            // Validated 1-15
            self.index
        } else {
            0
        };
        T::dpram().ep_in_buffer_control(index)
    }

    /// Give the buffer to the USB controller only after the rest of its
    /// control fields have crossed into the USB clock domain.
    fn write_buffer_control(&self, f: impl FnOnce(&mut rp_pac::usb_dpram::regs::EpBufferControl)) {
        let reg = self.buffer_control();
        let mut value = Default::default();
        f(&mut value);

        let available = value.available(0);
        value.set_available(0, false);
        reg.write_value(value);

        if available {
            cortex_m::asm::delay(USB_CLOCK_DELAY_CYCLES);
            value.set_available(0, true);
            reg.write_value(value);
        }
    }

    /// Get endpoint control register
    fn ep_control(&self) -> EpControlReg {
        if Self::is_interrupt() {
            T::dpram().ep_in_control(self.index - 1)
        } else {
            T::dpram_epx_control()
        }
    }

    /// Get interrupt endpoint address control
    fn addr_endp_host(&self) -> AddrControlReg {
        assert!(Self::is_interrupt());
        T::regs().addr_endp_x(self.index - 1)
    }

    /// Interrupt endpoints get their own hardware; everything else shares EPX.
    fn is_interrupt() -> bool {
        E::ep_type() == EndpointType::Interrupt
    }

    /// Wait for buffer to be available
    /// Returns stall status
    async fn wait_available(&self) -> bool {
        trace!("CHANNEL {} WAIT AVAILABLE", self.index);
        poll_fn(|cx| {
            // Both IN and OUT endpoints use IN registers on rp2040 in host mode
            self.waker().register(cx.waker());

            let reg = self.buffer_control().read();

            // A dedicated endpoint is always ours; only EPX needs the ownership test.
            if Self::is_interrupt() || self.can_run_epx_transaction() {
                self.clear_sie_status();
            }

            // FIXME: Stall derived from other place
            match reg.available(0) {
                true => Poll::Pending,
                false => Poll::Ready(false),
            }
        })
        .await
    }

    /// Mark this channel as currently used and configure endpoint type
    ///
    /// Call once on creation for interrupt pipe
    fn set_current(&self) {
        let regs = T::regs();
        trace!(
            "SET CURRENT: {:?} CHANNEL {}: dev: {}, ep: {}, max_packet: {}, preamble: {}",
            E::ep_type(),
            self.index,
            self.dev_addr,
            self.ep_addr,
            self.max_packet_size,
            self.pre
        );
        if Self::is_interrupt() {
            self.ep_control().write(|w| {
                w.set_endpoint_type(EpControlEndpointType::Interrupt);
                w.set_interrupt_per_buff(true);

                // `host_poll_interval` (bits 16:25) has no PAC accessor and counts from
                // zero, so clamp: a descriptor may declare an interval of 0.
                let interval = self.interval.max(1) as u32 - 1;
                w.0 |= interval << 16;

                w.set_buffer_address(self.buf.addr);
                w.set_enable(true);
            });

            // FIXME: What is this for?
            regs.sie_ctrl().modify(|w| w.set_sof_sync(true));

            self.addr_endp_host().write(|w| {
                w.set_address(self.dev_addr);
                w.set_endpoint(self.ep_addr);
                // FIXME: INTERRUPT OUT?
                w.set_intep_dir(D::is_out());
                w.set_intep_preamble(self.pre)
            });
        } else {
            T::host_state().epx.current_channel.store(self.index, Ordering::Relaxed);

            T::regs().addr_endp().write(|w| {
                w.set_address(self.dev_addr);
                w.set_endpoint(self.ep_addr);
            });

            self.ep_control().modify(|w| {
                w.set_enable(true);
                w.set_interrupt_per_buff(true);
                w.set_buffer_address(self.buf.addr);

                let epty = match E::ep_type() {
                    EndpointType::Control => EpControlEndpointType::Control,
                    EndpointType::Isochronous => EpControlEndpointType::Isochronous,
                    EndpointType::Bulk => EpControlEndpointType::Bulk,
                    EndpointType::Interrupt => EpControlEndpointType::Interrupt,
                };

                w.set_endpoint_type(epty);
            });

            regs.sie_ctrl().modify(|w| w.set_preamble_en(self.pre));
        }
    }

    /// EPX only.
    fn epx_transaction_guard(&self) -> EpxTransactionGuard<T> {
        debug_assert!(!Self::is_interrupt());
        EpxTransactionGuard {
            state: T::host_state(),
            index: self.index,
            ep_control: self.ep_control(),
            buffer_control: self.buffer_control(),
            transaction_active: false,
            _phantom: PhantomData,
        }
    }

    /// Reload interrupt channel buffer register
    fn interrupt_reload(&mut self) {
        assert!(E::ep_type() == EndpointType::Interrupt);
        self.write_buffer_control(|w| {
            w.set_last(0, true);
            w.set_pid(0, self.pid);
            w.set_full(0, false);
            w.set_reset(true);
            w.set_length(0, self.max_packet_size);
            w.set_available(0, true);
        });

        cortex_m::asm::delay(USB_CLOCK_DELAY_CYCLES);
        T::regs().int_ep_ctrl().modify(|w| {
            w.set_int_ep_active(w.int_ep_active() | 1 << (self.index - 1));
        });
    }

    /// One packet from a dedicated interrupt endpoint, which the pipe owns for the read.
    async fn polled_read(&mut self, buf: &mut [u8]) -> Result<usize, PipeError> {
        self.wait_available().await;
        self.set_current();

        let ctrl = self.buffer_control().read();
        if ctrl.full(0) {
            // A packet arrived while nobody was reading. Take it as it stands:
            // re-arming would clear `full` and discard what the device already sent.
        } else if ctrl.available(0) {
            // Already armed, so the controller owns the buffer and may fill it at
            // any moment. Writing to it here would race that; just wait.
            trace!("CHANNEL {} WAIT FOR INTERRUPT", self.index);
            self.wait_available().await;
        } else {
            // Idle: not armed and holding nothing, so the controller cannot be
            // touching the buffer and it is safe to program.
            trace!("CHANNEL {} ARM INTERRUPT", self.index);
            self.interrupt_reload();
            self.wait_available().await;
        }

        let rx_len = self.buffer_control().read().length(0) as usize;
        trace!("CHANNEL {} READ DONE, rx_len = {}", self.index, rx_len);
        if rx_len > buf.len() {
            return Err(PipeError::BufferOverflow);
        }
        self.buf.read(&mut buf[..rx_len]);

        self.advance_pid();
        self.interrupt_reload();
        Ok(rx_len)
    }

    /// Read over EPX until the caller's buffer fills or the device sends a short packet.
    async fn epx_read(&mut self, buf: &mut [u8]) -> Result<usize, PipeError> {
        let mut count: usize = 0;
        loop {
            trace!("CHANNEL {} START READ, len = {}", self.index, buf.len());
            let packet_len = core::cmp::min(buf.len() - count, self.max_packet_size as usize);
            let rx_len = self.transfer_in_packet(packet_len as u16, self.pid).await?;
            self.advance_pid();

            let free = &mut buf[count..];
            trace!("CHANNEL {} READ DONE, rx_len = {}", self.index, rx_len);
            if rx_len > free.len() {
                return Err(PipeError::BufferOverflow);
            }
            self.buf.read(&mut free[..rx_len]);
            count += rx_len;

            // A transfer ends at the caller's buffer or at a packet shorter than the max.
            if count == buf.len() || rx_len < self.max_packet_size as usize {
                return Ok(count);
            }
        }
    }

    fn advance_pid(&mut self) {
        if E::ep_type() != EndpointType::Isochronous {
            self.pid = !self.pid;
        }
    }

    /// Clear buffer interrupt bit
    fn clear_sie_status(&self) {
        if Self::is_interrupt() {
            T::regs().buff_status().write_clear(|w| w.0 = 0b11 << self.index * 2);
        } else {
            T::regs().buff_status().write_clear(|w| w.0 = 0b11);
        }
    }

    /// Send SETUP packet
    ///
    /// WARNING: This flips PID
    async fn send_setup(&mut self, setup: &[u8; 8]) -> Result<(), PipeError> {
        trace!("SEND SETUP");
        self.run_epx_transaction(|s| s.set_setup_packet(setup)).await?;
        self.pid = true;

        Ok(())
    }

    /// Send status packet
    async fn control_status(&mut self, active_direction_out: bool) -> Result<(), PipeError> {
        // Status packet always have DATA1
        trace!("SEND STATUS");
        self.run_epx_transaction(|s| {
            if active_direction_out {
                s.set_data_in(0, true);
            } else {
                s.set_data_out(&[], true);
            }
        })
        .await?;
        self.pid = false;

        Ok(())
    }
}

impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> UsbPipe<E, D> for Channel<'d, T, E, D> {
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsIn,
    {
        trace!("CONTROL IN: {:?}", setup);
        let length = u16::from_le_bytes([setup[6], setup[7]]) as usize;
        if length > buf.len() {
            return Err(PipeError::BufferOverflow);
        }

        self.start_control_timeout(length != 0);
        let result = async {
            self.send_setup(setup).await?;
            let read = if length > 0 {
                self.request_in(&mut buf[..length]).await?
            } else {
                0
            };
            self.control_status(false).await?;
            Ok(read)
        }
        .await;
        self.control_timeout_us = None;
        result
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        E: pipe::IsControl,
        D: pipe::IsOut,
    {
        trace!("CONTROL OUT: {:?}", setup);
        let length = u16::from_le_bytes([setup[6], setup[7]]) as usize;
        if length > buf.len() {
            return Err(PipeError::BufferOverflow);
        }

        self.start_control_timeout(length != 0);
        let result = async {
            self.send_setup(setup).await?;
            if length > 0 {
                self.request_out(&buf[..length], false).await?;
            }
            self.control_status(true).await?;
            Ok(())
        }
        .await;
        self.control_timeout_us = None;
        result
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        if Self::is_interrupt() {
            self.polled_read(buf).await
        } else {
            self.epx_read(buf).await
        }
    }

    async fn request_out(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let mut count = 0;

        let res = loop {
            trace!("CHANNEL {} START WRITE", self.index);
            let packet = self.transfer_out_packet(&buf[count..], self.pid).await?;
            self.advance_pid();

            trace!("WRITE DONE, tx_len = {}", packet);

            count += packet;

            if count == buf.len() {
                if packet == self.max_packet_size as usize && ensure_transaction_end {
                    trace!("CHANNEL {} START ZLP WRITE", self.index);
                    self.transfer_out_packet(&[], self.pid).await?;
                    self.advance_pid();
                    trace!("ZLP WRITE DONE");
                }
                break Ok(());
            }
        };

        res
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig) {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self) {
        self.pid = false;
    }
}

impl<'d, T: SealedHostInstance, E, D> Channel<'d, T, E, D> {}

impl<'d, T: SealedHostInstance, E, D> Drop for Channel<'d, T, E, D> {
    fn drop(&mut self) {
        if self.index < EP_COUNT {
            // Disarm and clear stale state so the interrupt slot can be reused safely.
            let regs = T::regs();
            let dpram = T::dpram();

            regs.int_ep_ctrl().modify(|w| {
                w.set_int_ep_active(w.int_ep_active() & !(1 << (self.index - 1)));
            });
            dpram.ep_in_control(self.index - 1).write(|w| w.0 = 0);
            dpram.ep_in_buffer_control(self.index).write(|w| w.0 = 0);
            regs.buff_status().write_clear(|w| w.0 = 0b11 << (self.index * 2));

            let state = T::host_state();
            critical_section::with(|_| {
                let pipes = &state.polled.allocated_pipes;
                pipes.store(pipes.load(Ordering::Relaxed) & !(1 << self.index), Ordering::Relaxed);
            });
        } else {
            let state = T::host_state();
            // Return the EPX buffer and the pipe slot to the pool.
            free_epx_mem(state, self.buf.addr, self.buf.len);
            state.epx.discard_epx_outcome(self.epx_slot());
            critical_section::with(|_| {
                let epx = &state.epx.allocated_epx;
                epx.store(
                    epx.load(Ordering::Relaxed) & !(1 << (self.epx_slot())),
                    Ordering::Relaxed,
                );
            });
            debug!(
                "EPX pipe FREE  slot {} (bitset {:04x})",
                self.epx_slot(),
                state.epx.allocated_epx.load(Ordering::Relaxed)
            );
        }
    }
}

/// Pipe allocator handle for [`Driver`].
pub struct Allocator<'d, T: Instance> {
    phantom: PhantomData<&'d T>,
}

impl<'d, T: Instance> Clone for Allocator<'d, T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<'d, T: Instance> Copy for Allocator<'d, T> {}

impl<'d, T: SealedHostInstance> UsbHostAllocator<'d> for Allocator<'d, T> {
    type Pipe<E: pipe::Type, D: pipe::Direction> = Channel<'d, T, E, D>;

    fn alloc_pipe<E: pipe::Type, D: pipe::Direction>(
        &self,
        dev_addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<E, D>, HostError> {
        let state = T::host_state();
        let pre = split_to_pre(split);
        if E::ep_type() == EndpointType::Interrupt {
            let free_index = critical_section::with(|_| {
                let alloc = state.polled.allocated_pipes.load(Ordering::Relaxed);
                if let Some(idx) = (1..EP_COUNT).find(|i| alloc & (1 << i) == 0) {
                    state
                        .polled
                        .allocated_pipes
                        .store(alloc | (1 << idx), Ordering::Relaxed);
                    Ok(idx as u8)
                } else {
                    Err(HostError::OutOfPipes)
                }
            })?;
            // Fixed layout: pipe index 1..EP_COUNT maps to block 0..EP_COUNT-1.
            let addr = DPRAM_DATA_OFFSET + (free_index as u16 - 1) * EPX_BLOCK_SIZE as u16;

            Ok(Channel::new(free_index as _, addr, 64, endpoint, dev_addr, pre))
        } else {
            let index = critical_section::with(|_| {
                let alloc = state.epx.allocated_epx.load(Ordering::Relaxed);
                let slot = alloc.trailing_ones() as usize;
                if slot >= EPX_MAX_PIPES {
                    return Err(HostError::OutOfPipes);
                }
                state.epx.allocated_epx.store(alloc | (1 << slot), Ordering::Relaxed);
                Ok(EP_COUNT + slot)
            })?;
            let slot = index - EP_COUNT;
            // One buffer per pipe: a parked transfer's data must survive another pipe
            // taking EPX in the meantime.
            let len = (blocks_for(endpoint.max_packet_size) * EPX_BLOCK_SIZE) as u16;
            let addr = match alloc_epx_mem(state, len) {
                Ok(addr) => addr,
                Err(()) => {
                    // Hand back the slot claimed above, or repeated buffer failures
                    // would exhaust the bitset and report `OutOfPipes` instead.
                    critical_section::with(|_| {
                        let epx = &state.epx.allocated_epx;
                        epx.store(epx.load(Ordering::Relaxed) & !(1 << slot), Ordering::Relaxed);
                    });

                    return Err(HostError::InsufficientMemory);
                }
            };
            debug!(
                "EPX pipe ALLOC slot {} (bitset {:04x}) buf {:#x}+{}",
                slot,
                state.epx.allocated_epx.load(Ordering::Relaxed),
                addr,
                len
            );

            Ok(Channel::new(index, addr, len, endpoint, dev_addr, pre))
        }
    }
}

impl<'d, T: SealedHostInstance> UsbHostController<'d> for Driver<'d, T> {
    type Allocator = Allocator<'d, T>;

    fn allocator(&self) -> Self::Allocator {
        Allocator { phantom: PhantomData }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        let is_connected = |status: u8| match status {
            0b01 | 0b10 => true,
            _ => false,
        };

        let was = self.connected;

        // Clear interrupt status
        T::regs().sie_status().write_clear(|w| {
            w.set_speed(0b11);
        });

        // Enable conn/dis irq
        T::regs().inte().modify(|w| {
            w.set_host_conn_dis(true);
        });
        let ev = poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            let now = T::regs().sie_status().read().speed();
            let speed_now: DeviceEvent = match now {
                0b01 => DeviceEvent::Connected(Speed::Low),
                0b10 => DeviceEvent::Connected(Speed::Full),
                _ => DeviceEvent::Disconnected,
            };
            match (was, is_connected(now)) {
                (true, false) => Poll::Ready(DeviceEvent::Disconnected),
                (false, true) => Poll::Ready(speed_now),
                _ => Poll::Pending,
            }
        })
        .await;

        self.connected = matches!(ev, DeviceEvent::Connected(_));

        // Per the `UsbHostController` contract, reset before reporting the attach so the
        // device leaves Powered for Default (USB 2.0 §9.1.2). Full-speed only, so no chirp.
        if matches!(ev, DeviceEvent::Connected(_)) {
            self.bus_reset().await;
        }
        ev
    }

    async fn bus_reset(&mut self) {
        T::regs().sie_ctrl().modify(|w| {
            w.set_reset_bus(true);
        });

        embassy_time::Timer::after_millis(ROOT_RESET_MS).await;

        T::regs().sie_ctrl().modify(|w| {
            w.set_reset_bus(false);
        });

        embassy_time::Timer::after_millis(RESET_RECOVERY_MS).await;
    }
}

/// Service the RP235x stop-on-NAK interrupt, handing EPX to whoever is queued.
/// Returns `true` when it consumed the interrupt; RP2040 has no such interrupt.
fn on_nak_stop<T: SealedHostInstance>() -> bool {
    #[cfg(feature = "_rp235x")]
    {
        let regs = T::regs();
        if T::regs().ints().read().epx_stopped_on_nak() {
            let index = T::host_state().epx.current_channel.load(Ordering::Acquire);

            // Acknowledge, stop asking, then hand the endpoint over.
            regs.nak_poll().modify(|w| w.set_epx_stopped_on_nak(true));
            disarm_epx_yield::<T>();
            let yielded = yield_epx::<T>(index);
            if !yielded {
                trace!("USB IRQ: EPx completed as it was stopped; keeping the packet");
            }

            trace!("USB IRQ: EPx stopped on NAK, yielded channel {}", index);
            BUS_WAKER.wake();
            return true;
        }
    }
    false
}

/// Service the SOF interrupt. RP235x only silences it; RP2040 uses frame boundaries
/// to stop a NAKing holder so a queued pipe can take EPX.
fn on_host_sof<T: SealedHostInstance>() -> &'static str {
    #[allow(unused)]
    let regs = T::regs();
    #[cfg(feature = "_rp235x")]
    {
        // Prevent nonstop SOF interrupt
        T::regs().inte().write_clear(|w| w.set_host_sof(true));
        "sof"
    }
    #[cfg(feature = "rp2040")]
    {
        // Reading SOF_RD acknowledges HOST_SOF on RP2040.
        let _ = regs.sof_rd().read();
        let state = T::host_state();
        let index = state.epx.current_channel.load(Ordering::Acquire);

        if state.epx.with_arbiter(|a| a.waiting) == 0 {
            // Nobody is queued for EPX, so there is nothing to yield to.
            disarm_epx_yield::<T>();
            "sof idle"
        } else if index == 0 {
            // EPX is free; the queue will claim it without a yield.
            state.epx.with_arbiter(|a| a.switch_requested = false);
            "sof free"
        } else if state.epx.with_arbiter(|a| a.switch_requested) {
            // Two frame boundaries with no completion between them means the
            // holder is retrying NAKs: safe to stop it here.
            regs.sie_ctrl().modify(|w| w.set_stop_trans(true));
            while regs.sie_ctrl().read().stop_trans() {}

            state.epx.with_arbiter(|a| a.switch_requested = false);
            if yield_epx::<T>(index) {
                "sof yielded EPx"
            } else {
                "sof stop raced a completion; packet kept"
            }
        } else {
            state.epx.with_arbiter(|a| a.switch_requested = true);
            "sof armed EPx yield"
        }
    }
}

/// USB interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _usb: PhantomData<T>,
}

impl<T: SealedHostInstance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let ints = regs.ints().read();

        // Hand EPX over if the controller stopped it at a NAK boundary.
        if on_nak_stop::<T>() {
            return;
        }

        let ev = {
            if ints.host_conn_dis() {
                regs.inte().write_clear(|w| w.set_host_conn_dis(true));
                match regs.sie_status().read().speed() {
                    0b01 => "attached low speed",
                    0b10 => "attached full speed",
                    _ => "detached",
                }
            } else if ints.host_resume() {
                regs.sie_status().write_clear(|w| w.set_resume(true));
                "resume"
            } else if ints.stall() {
                regs.inte().write_clear(|w| w.set_stall(true));
                T::host_state().epx.wake_current_epx();
                "stall"
            } else if ints.error_rx_overflow() {
                regs.inte().write_clear(|w| w.set_error_rx_overflow(true));
                T::host_state().epx.wake_current_epx();
                "rx overflow"
            } else if ints.trans_complete() {
                regs.inte().write_clear(|w| w.set_trans_complete(true));
                note_epx_progress::<T>();
                T::host_state().epx.wake_current_epx();
                "transaction complete"
            } else if ints.error_rx_timeout() {
                regs.inte().write_clear(|w| w.set_error_rx_timeout(true));
                T::host_state().epx.wake_current_epx();
                "rx timeout"
            } else if ints.buff_status() {
                let status = regs.buff_status().read().0;
                // Bits 0 and 1 are EPX's IN/OUT pair; from bit 2 up they are the
                // dedicated interrupt endpoints, two bits per endpoint. Only interrupt IN
                // gets a dedicated endpoint, so of each pair only the IN bit is ever armed.
                if status & 0b11 != 0 {
                    regs.buff_status().write_clear(|w| w.0 = status & 0b11);
                    trace!("USB IRQ: EPx");
                    T::host_state().epx.wake_current_epx();
                }

                for n in 1..EP_COUNT {
                    if status & (1 << (n * 2)) != 0 {
                        regs.buff_status().write_clear(|w| w.0 = 0b11 << (n * 2));
                        trace!("USB IRQ: Interrupt EP {}", n);
                        EP_IN_WAKERS[n].wake();
                    }
                }
                "^^^"
            } else if ints.error_crc() {
                regs.sie_status().write_clear(|w| w.set_crc_error(true));
                record_epx_error::<T>(EpxError::BadResponse);
                "crc error"
            } else if ints.error_bit_stuff() {
                regs.sie_status().write_clear(|w| w.set_bit_stuff_error(true));
                record_epx_error::<T>(EpxError::BadResponse);
                "bit stuff error"
            } else if ints.error_data_seq() {
                regs.sie_status().write_clear(|w| w.set_data_seq_error(true));
                record_epx_error::<T>(EpxError::DataToggle);
                "data sequence error"
            } else if ints.host_sof() {
                on_host_sof::<T>()
            } else {
                "???"
            }
        };

        trace!("USB IRQ: {:08x} :: {}", ints.0, ev);

        BUS_WAKER.wake();
    }
}
