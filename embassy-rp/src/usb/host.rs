use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::Poll;

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

use epx::{
    EPX_BLOCK_SIZE, EPX_MAX_PIPES, EpxError, EpxState, note_epx_progress, on_host_sof, on_nak_stop, record_epx_error,
    wake_current,
};

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
struct InterruptPipeState {
    /// Bitset indexed by endpoint index (1..EP_COUNT).
    allocated: AtomicU16,
    /// Pipes whose cancelled transfer completed before its data toggle was advanced.
    toggle_owed: AtomicU16,
    /// Interrupt IN endpoints with a receive buffer armed.
    armed_in: AtomicU16,
    /// Interrupt IN endpoints charged with the SIE's global receive timeout.
    timed_out: AtomicU16,
}

impl InterruptPipeState {
    const fn new() -> Self {
        Self {
            allocated: AtomicU16::new(0),
            toggle_owed: AtomicU16::new(0),
            armed_in: AtomicU16::new(0),
            timed_out: AtomicU16::new(0),
        }
    }

    fn reset(&self) {
        self.allocated.store(0, Ordering::Relaxed);
        self.toggle_owed.store(0, Ordering::Relaxed);
        self.armed_in.store(0, Ordering::Relaxed);
        self.timed_out.store(0, Ordering::Relaxed);
    }

    fn allocate(&self) -> Result<usize, HostError> {
        critical_section::with(|_| {
            let allocated = self.allocated.load(Ordering::Relaxed);
            let index = (1..EP_COUNT)
                .find(|index| allocated & (1 << index) == 0)
                .ok_or(HostError::OutOfPipes)?;
            self.allocated.store(allocated | (1 << index), Ordering::Relaxed);
            Ok(index)
        })
    }

    fn free(&self, index: usize) {
        critical_section::with(|_| {
            let bit = 1 << index;
            let allocated = self.allocated.load(Ordering::Relaxed);
            self.allocated.store(allocated & !bit, Ordering::Relaxed);
            let owed = self.toggle_owed.load(Ordering::Relaxed);
            self.toggle_owed.store(owed & !bit, Ordering::Relaxed);
            let armed = self.armed_in.load(Ordering::Relaxed);
            self.armed_in.store(armed & !bit, Ordering::Relaxed);
            let timed_out = self.timed_out.load(Ordering::Relaxed);
            self.timed_out.store(timed_out & !bit, Ordering::Relaxed);
        });
    }

    fn mark_armed_in(&self, index: usize) {
        critical_section::with(|_| {
            let armed = self.armed_in.load(Ordering::Relaxed);
            self.armed_in.store(armed | (1 << index), Ordering::Relaxed);
        });
    }

    fn mark_completed(&self, index: usize) {
        critical_section::with(|_| {
            let armed = self.armed_in.load(Ordering::Relaxed);
            self.armed_in.store(armed & !(1 << index), Ordering::Relaxed);
        });
    }

    /// Charge a global receive timeout to every interrupt IN endpoint that was armed.
    /// The hardware does not identify which endpoint caused it.
    fn record_timeout(&self) {
        let candidates = critical_section::with(|_| {
            let candidates = self.armed_in.load(Ordering::Relaxed);
            let timed_out = self.timed_out.load(Ordering::Relaxed);
            self.timed_out.store(timed_out | candidates, Ordering::Relaxed);
            self.armed_in.store(0, Ordering::Relaxed);
            candidates
        });
        for index in 1..EP_COUNT {
            if candidates & (1 << index) != 0 {
                EP_IN_WAKERS[index].wake();
            }
        }
    }

    fn take_timeout(&self, index: usize) -> bool {
        critical_section::with(|_| {
            let bit = 1 << index;
            let timed_out = self.timed_out.load(Ordering::Relaxed);
            self.timed_out.store(timed_out & !bit, Ordering::Relaxed);
            timed_out & bit != 0
        })
    }

    fn mark_toggle_owed(&self, index: usize) {
        critical_section::with(|_| {
            let owed = self.toggle_owed.load(Ordering::Relaxed);
            self.toggle_owed.store(owed | (1 << index), Ordering::Relaxed);
        });
    }

    fn take_toggle_owed(&self, index: usize) -> bool {
        critical_section::with(|_| {
            let bit = 1 << index;
            let owed = self.toggle_owed.load(Ordering::Relaxed);
            self.toggle_owed.store(owed & !bit, Ordering::Relaxed);
            owed & bit != 0
        })
    }

    fn clear_toggle_owed(&self, index: usize) {
        critical_section::with(|_| {
            let owed = self.toggle_owed.load(Ordering::Relaxed);
            self.toggle_owed.store(owed & !(1 << index), Ordering::Relaxed);
        });
    }
}

/// Per-instance state shared between [`Driver`], [`Allocator`] and [`Channel`].
pub struct HostState {
    epx: EpxState,
    interrupt_pipes: InterruptPipeState,
}

impl HostState {
    /// Create a new, reset host state.
    pub const fn new() -> Self {
        Self {
            epx: EpxState::new(),
            interrupt_pipes: InterruptPipeState::new(),
        }
    }

    fn reset(&self) {
        self.epx.reset();
        self.interrupt_pipes.reset();
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
            // The register carries the endpoint number; direction is configured separately.
            ep_addr: u8::from(ep_info.addr) & 0x0f,
            interval: ep_info.interval_ms,
            pid: false,
            pre,
            timeout: default_timeout(),
            control_timeout_us: None,
        }
    }
}

type BufferControlReg = rp_pac::common::Reg<rp_pac::usb_dpram::regs::EpBufferControl, rp_pac::common::RW>;
type EpControlReg = rp_pac::common::Reg<rp_pac::usb_dpram::regs::EpControl, rp_pac::common::RW>;
type AddrControlReg = rp_pac::common::Reg<rp_pac::usb::regs::AddrEndpX, rp_pac::common::RW>;

/// Stops a dedicated interrupt endpoint if its transfer future is cancelled.
struct InterruptTransferGuard<T: SealedHostInstance> {
    index: usize,
    is_out: bool,
    buffer_control: BufferControlReg,
    active: bool,
    _phantom: PhantomData<T>,
}

impl<T: SealedHostInstance> InterruptTransferGuard<T> {
    fn arm(&mut self) {
        self.active = true;
    }

    fn disarm(&mut self) {
        self.active = false;
    }
}

impl<T: SealedHostInstance> Drop for InterruptTransferGuard<T> {
    fn drop(&mut self) {
        if !self.active {
            return;
        }

        let regs = T::regs();
        regs.int_ep_ctrl().modify(|w| {
            w.set_int_ep_active(w.int_ep_active() & !(1 << (self.index - 1)));
        });

        let buffer = self.buffer_control.read();
        let moved = !buffer.available(0) && if self.is_out { !buffer.full(0) } else { buffer.full(0) };
        T::host_state().interrupt_pipes.mark_completed(self.index);
        if moved {
            T::host_state().interrupt_pipes.mark_toggle_owed(self.index);
        }

        self.buffer_control.modify(|w| {
            w.set_available(0, false);
            w.set_full(0, false);
        });
        regs.buff_status().write_clear(|w| w.0 = 0b11 << (self.index * 2));
    }
}

impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> Channel<'d, T, E, D> {
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

    fn ep_control(&self) -> EpControlReg {
        if Self::is_interrupt() {
            T::dpram().ep_in_control(self.index - 1)
        } else {
            T::dpram_epx_control()
        }
    }

    fn addr_endp_host(&self) -> AddrControlReg {
        assert!(Self::is_interrupt());
        T::regs().addr_endp_x(self.index - 1)
    }

    fn is_interrupt() -> bool {
        E::ep_type() == EndpointType::Interrupt
    }

    /// Wait for the controller to release a dedicated interrupt endpoint's buffer.
    async fn wait_interrupt_buffer(&self) -> Result<(), PipeError> {
        trace!("CHANNEL {} WAIT AVAILABLE", self.index);
        if D::is_out() {
            return poll_fn(|cx| self.poll_interrupt_buffer(cx)).await;
        }

        // RX timeout is not delivered through INTS, so sample its global status bit at
        // the endpoint's polling interval.
        let interval = embassy_time::Duration::from_millis(self.interval.max(1) as u64);
        loop {
            match embassy_time::with_timeout(interval, poll_fn(|cx| self.poll_interrupt_buffer(cx))).await {
                Ok(result) => return result,
                Err(_) => {
                    let state = T::host_state();
                    // Only attribute the global status while the shared endpoint is idle.
                    if state.epx.is_idle() && T::regs().sie_status().read().rx_timeout() {
                        T::regs().sie_status().write_clear(|w| w.set_rx_timeout(true));
                        state.interrupt_pipes.record_timeout();
                    }
                }
            }
        }
    }

    fn poll_interrupt_buffer(&self, cx: &mut core::task::Context<'_>) -> Poll<Result<(), PipeError>> {
        // Both directions use the IN endpoint registers in host mode.
        EP_IN_WAKERS[self.index].register(cx.waker());

        if T::host_state().interrupt_pipes.take_timeout(self.index) {
            T::regs().int_ep_ctrl().modify(|w| {
                w.set_int_ep_active(w.int_ep_active() & !(1 << (self.index - 1)));
            });
            self.buffer_control().modify(|w| {
                w.set_available(0, false);
                w.set_full(0, false);
            });
            T::regs().buff_status().write_clear(|w| w.0 = 0b11 << self.index * 2);
            return Poll::Ready(Err(PipeError::Timeout));
        }

        let reg = self.buffer_control().read();
        T::regs().buff_status().write_clear(|w| w.0 = 0b11 << self.index * 2);

        match reg.available(0) {
            true => Poll::Pending,
            false => {
                T::host_state().interrupt_pipes.mark_completed(self.index);
                Poll::Ready(Ok(()))
            }
        }
    }

    /// Restore the dedicated endpoint configuration before each transfer.
    fn configure_interrupt(&self) {
        debug_assert!(Self::is_interrupt());
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
        self.ep_control().write(|w| {
            w.set_endpoint_type(EpControlEndpointType::Interrupt);
            w.set_interrupt_per_buff(true);

            // `host_poll_interval` (bits 16:25) has no PAC accessor and counts from zero.
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
            w.set_intep_dir(D::is_out());
            w.set_intep_preamble(self.pre)
        });
    }

    fn interrupt_transfer_guard(&self) -> InterruptTransferGuard<T> {
        debug_assert!(Self::is_interrupt());
        InterruptTransferGuard {
            index: self.index,
            is_out: D::is_out(),
            buffer_control: self.buffer_control(),
            active: false,
            _phantom: PhantomData,
        }
    }

    /// Apply a toggle consumed by a packet that completed during cancellation.
    fn settle_interrupt_toggle(&mut self) {
        if T::host_state().interrupt_pipes.take_toggle_owed(self.index) {
            self.advance_pid();
        }
    }

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
        T::host_state().interrupt_pipes.mark_armed_in(self.index);
        T::regs().int_ep_ctrl().modify(|w| {
            w.set_int_ep_active(w.int_ep_active() | 1 << (self.index - 1));
        });
    }

    /// Load an interrupt OUT buffer and arm it for the next poll.
    fn interrupt_send(&mut self, data: &[u8]) -> usize {
        assert!(Self::is_interrupt() && D::is_out());
        let chunk = &data[..data.len().min(self.max_packet_size as usize)];
        self.buf.write(chunk);

        self.write_buffer_control(|w| {
            w.set_last(0, chunk.len() == data.len());
            w.set_pid(0, self.pid);
            w.set_full(0, true);
            w.set_reset(true);
            w.set_length(0, chunk.len() as u16);
            w.set_available(0, true);
        });

        cortex_m::asm::delay(USB_CLOCK_DELAY_CYCLES);
        T::regs().int_ep_ctrl().modify(|w| {
            w.set_int_ep_active(w.int_ep_active() | 1 << (self.index - 1));
        });

        chunk.len()
    }

    /// Read one packet from a dedicated interrupt endpoint.
    async fn interrupt_in_read(&mut self, buf: &mut [u8]) -> Result<usize, PipeError> {
        self.wait_interrupt_buffer().await?;
        self.configure_interrupt();
        self.settle_interrupt_toggle();
        let mut guard = self.interrupt_transfer_guard();
        guard.arm();

        let ctrl = self.buffer_control().read();
        if ctrl.full(0) {
            // A packet arrived while nobody was reading. Take it as it stands:
            // re-arming would clear `full` and discard what the device already sent.
        } else if ctrl.available(0) {
            // Already armed, so the controller owns the buffer and may fill it at
            // any moment. Writing to it here would race that; just wait.
            trace!("CHANNEL {} WAIT FOR INTERRUPT", self.index);
            self.wait_interrupt_buffer().await?;
        } else {
            // Idle: not armed and holding nothing, so the controller cannot be
            // touching the buffer and it is safe to program.
            trace!("CHANNEL {} ARM INTERRUPT", self.index);
            self.interrupt_reload();
            self.wait_interrupt_buffer().await?;
        }

        let rx_len = self.buffer_control().read().length(0) as usize;
        trace!("CHANNEL {} READ DONE, rx_len = {}", self.index, rx_len);
        if rx_len > buf.len() {
            return Err(PipeError::BufferOverflow);
        }
        self.buf.read(&mut buf[..rx_len]);
        self.advance_pid();
        self.interrupt_reload();
        guard.disarm();
        Ok(rx_len)
    }

    /// Write to a dedicated interrupt endpoint, one packet per poll.
    async fn interrupt_out_write(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError> {
        self.wait_interrupt_buffer().await?;
        self.configure_interrupt();
        self.settle_interrupt_toggle();
        let mut guard = self.interrupt_transfer_guard();
        guard.arm();

        // Run once so an empty buffer sends a ZLP.
        let mut count = 0;
        let mut packet;
        loop {
            trace!("CHANNEL {} ARM INTERRUPT OUT", self.index);
            packet = self.interrupt_send(&buf[count..]);
            self.wait_interrupt_buffer().await?;
            self.advance_pid();
            count += packet;
            if count >= buf.len() {
                break;
            }
        }

        if ensure_transaction_end && packet == self.max_packet_size as usize {
            trace!("CHANNEL {} ARM INTERRUPT OUT ZLP", self.index);
            self.interrupt_send(&[]);
            self.wait_interrupt_buffer().await?;
            self.advance_pid();
        }

        guard.disarm();
        Ok(())
    }

    /// Read over EPX until the caller's buffer fills or the device sends a short packet.
    async fn epx_read(&mut self, buf: &mut [u8]) -> Result<usize, PipeError> {
        let mut count: usize = 0;
        let res = loop {
            trace!("CHANNEL {} START READ, len = {}", self.index, buf.len());
            let packet_len = core::cmp::min(buf.len() - count, self.max_packet_size as usize);
            let rx_len = self.transfer_in_packet(packet_len as u16, self.pid).await?;
            self.advance_pid();

            let free = &mut buf[count..];
            trace!("CHANNEL {} READ DONE, rx_len = {}", self.index, rx_len);
            if rx_len > free.len() {
                break Err(PipeError::BufferOverflow);
            }

            self.buf.read(&mut free[..rx_len]);
            count += rx_len;
            if count == buf.len() || rx_len < self.max_packet_size as usize {
                break Ok(count);
            }
        };

        res
    }

    /// Write over EPX until the caller's buffer is drained.
    async fn epx_write(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError> {
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

    fn advance_pid(&mut self) {
        if E::ep_type() != EndpointType::Isochronous {
            self.pid = !self.pid;
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
            self.interrupt_in_read(buf).await
        } else {
            self.epx_read(buf).await
        }
    }

    async fn request_out(&mut self, buf: &[u8], ensure_transaction_end: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        if Self::is_interrupt() {
            self.interrupt_out_write(buf, ensure_transaction_end).await
        } else {
            self.epx_write(buf, ensure_transaction_end).await
        }
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig) {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self) {
        self.pid = false;
        if Self::is_interrupt() {
            T::host_state().interrupt_pipes.clear_toggle_owed(self.index);
        }
    }
}

impl<'d, T: SealedHostInstance, E, D> Drop for Channel<'d, T, E, D> {
    fn drop(&mut self) {
        if self.index < EP_COUNT {
            let regs = T::regs();
            let dpram = T::dpram();

            regs.int_ep_ctrl().modify(|w| {
                w.set_int_ep_active(w.int_ep_active() & !(1 << (self.index - 1)));
            });
            dpram.ep_in_control(self.index - 1).write(|w| w.0 = 0);
            dpram.ep_in_buffer_control(self.index).write(|w| w.0 = 0);
            regs.buff_status().write_clear(|w| w.0 = 0b11 << (self.index * 2));

            T::host_state().interrupt_pipes.free(self.index);
        } else {
            T::host_state().epx.free(self.epx_slot(), self.buf.addr, self.buf.len);
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
            if endpoint.max_packet_size == 0 || endpoint.max_packet_size as usize > EPX_BLOCK_SIZE {
                return Err(HostError::InvalidDescriptor);
            }
            let index = state.interrupt_pipes.allocate()?;
            // Fixed layout: pipe index 1..EP_COUNT maps to block 0..EP_COUNT-1.
            let addr = DPRAM_DATA_OFFSET + (index as u16 - 1) * EPX_BLOCK_SIZE as u16;

            Ok(Channel::new(
                index,
                addr,
                EPX_BLOCK_SIZE as u16,
                endpoint,
                dev_addr,
                pre,
            ))
        } else {
            let (index, addr, len) = state.epx.allocate(endpoint.max_packet_size)?;

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
                wake_current::<T>();
                "stall"
            } else if ints.error_rx_overflow() {
                regs.inte().write_clear(|w| w.set_error_rx_overflow(true));
                wake_current::<T>();
                "rx overflow"
            } else if ints.trans_complete() {
                regs.inte().write_clear(|w| w.set_trans_complete(true));
                note_epx_progress::<T>();
                wake_current::<T>();
                "transaction complete"
            } else if ints.error_rx_timeout() {
                regs.inte().write_clear(|w| w.set_error_rx_timeout(true));
                wake_current::<T>();
                "rx timeout"
            } else if ints.buff_status() {
                let status = regs.buff_status().read().0;
                // Bits 0 and 1 are EPX's IN/OUT pair; from bit 2 up they are the
                // dedicated interrupt endpoints, two bits per endpoint. Either direction
                // can signal, so clear both bits in the pair.
                if status & 0b11 != 0 {
                    regs.buff_status().write_clear(|w| w.0 = status & 0b11);
                    trace!("USB IRQ: EPx");
                    wake_current::<T>();
                }

                for n in 1..EP_COUNT {
                    if status & (0b11 << (n * 2)) != 0 {
                        regs.buff_status().write_clear(|w| w.0 = 0b11 << (n * 2));
                        T::host_state().interrupt_pipes.mark_completed(n);
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
