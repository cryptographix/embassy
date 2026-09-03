//! Arbitration and buffers for the shared non-interrupt endpoint.

use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU16, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::EndpointType;
use embassy_usb_driver::host::{HostError, PipeError, pipe};
use rp_pac::usb_dpram::vals::EpControlEndpointType;

#[cfg(feature = "_rp235x")]
use super::super::BUS_WAKER;
use super::super::{DPRAM_DATA_OFFSET, EP_COUNT, EP_MEMORY_SIZE};
use super::{BufferControlReg, Channel, EpControlReg, HostState, SIE_START_DELAY_CYCLES, SealedHostInstance};
use crate::RegExt;

// DPRAM layout - `EP_COUNT` blocks for control and interrupts, then remaining blocks
// allocatable to EPX pipes.
pub(super) const EPX_BLOCK_SIZE: usize = 64;

/// First EPX buffer after the control and interrupt buffers.
const EPX_BUFFER_OFFSET: u16 = DPRAM_DATA_OFFSET + ((EP_COUNT - 1) * EPX_BLOCK_SIZE) as u16;

/// Number of allocatable EPX blocks.
const EPX_NUM_BLOCKS: usize = (EP_MEMORY_SIZE - EPX_BUFFER_OFFSET as usize) / EPX_BLOCK_SIZE;

/// Maximum number of concurrently allocated EPX pipes.
pub(super) const EPX_MAX_PIPES: usize = 16;

/// NAK retry interval, stretched so a NAKing transfer cannot retry within the frame.
#[cfg(feature = "rp2040")]
const NAK_POLL_DELAY_YIELD: u16 = 300;

/// NAK retry interval outside a yield.
#[cfg(feature = "rp2040")]
const NAK_POLL_DELAY_NORMAL: u16 = 16;

/// Bits reserved per EPX pipe in [`EpxArbiter::error`].
const EPX_ERROR_BITS: usize = 4;

/// Mask of one pipe's error field.
const EPX_ERROR_MASK: u64 = (1 << EPX_ERROR_BITS) - 1;

/// Terminal errors the interrupt handler can record, packed [`EPX_ERROR_BITS`] wide.
/// Zero means no error, so the whole field clears to "nothing recorded".
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u64)]
pub(super) enum EpxError {
    BadResponse = 1,
    DataToggle = 2,
}

impl EpxError {
    fn from_code(code: u64) -> Option<Self> {
        match code {
            1 => Some(Self::BadResponse),
            2 => Some(Self::DataToggle),
            _ => None,
        }
    }
}

impl From<EpxError> for PipeError {
    fn from(error: EpxError) -> Self {
        match error {
            EpxError::BadResponse => PipeError::BadResponse,
            EpxError::DataToggle => PipeError::DataToggleError,
        }
    }
}

/// Who holds EPX, who is queued for it, and what the interrupt recorded for a
/// transfer that was stopped.
struct EpxArbiter {
    /// Bitset of EPX pipes whose transaction was stopped at a NAK boundary.
    yielded: u16,
    /// Bitset of EPX pipes queued in `acquire_epx`.
    waiting: u16,
    /// Whether EPX has a transaction armed. Framing errors come from the shared RX
    /// engine, so they are only charged to EPX while it is mid-transaction.
    armed: bool,
    /// Bitset of EPX pipes whose packet landed as the transfer was being stopped.
    /// `STOP_TRANS` suppresses `trans_complete`, so the completion is reported here.
    completed: u16,
    /// Terminal error per EPX pipe, [`EPX_ERROR_BITS`] wide. Clearing the status bit is
    /// what stops the interrupt re-firing, so the reason is carried here instead.
    error: u64,
    /// Slot that last held EPX. The queue is served starting after it, so a pipe
    /// that just released goes to the back.
    last: usize,
    /// RP2040 only: set on the first SOF of a contended EPX transfer, so the
    /// second SOF with no completion in between can stop it.
    #[cfg(feature = "rp2040")]
    switch_requested: bool,
}

impl EpxArbiter {
    /// Arbitration state with nothing queued, held or recorded.
    const fn new() -> Self {
        Self {
            yielded: 0,
            waiting: 0,
            armed: false,
            completed: 0,
            error: 0,
            last: 0,
            #[cfg(feature = "rp2040")]
            switch_requested: false,
        }
    }

    /// Slot whose turn it is to take a free EPX: the first queued pipe after the one
    /// that last held it. `None` when nobody is queued.
    fn turn(&self) -> Option<usize> {
        if self.waiting == 0 {
            return None;
        }
        (1..=EPX_MAX_PIPES)
            .map(|step| (self.last + step) % EPX_MAX_PIPES)
            .find(|slot| self.waiting & (1 << slot) != 0)
    }

    /// Queue a pipe for EPX, or take it out of the queue.
    fn set_waiting(&mut self, slot: usize, waiting: bool) {
        if waiting {
            self.waiting |= 1 << slot;
        } else {
            self.waiting &= !(1 << slot);
        }
    }

    fn mark_yielded(&mut self, slot: usize) {
        self.yielded |= 1 << slot;
    }

    fn take_yielded(&mut self, slot: usize) -> bool {
        let was = self.yielded & (1 << slot) != 0;
        self.yielded &= !(1 << slot);
        was
    }

    fn mark_completed(&mut self, slot: usize) {
        self.completed |= 1 << slot;
    }

    fn take_completed(&mut self, slot: usize) -> bool {
        let was = self.completed & (1 << slot) != 0;
        self.completed &= !(1 << slot);
        was
    }

    fn mark_error(&mut self, slot: usize, error: EpxError) {
        let shift = slot * EPX_ERROR_BITS;
        self.error = (self.error & !(EPX_ERROR_MASK << shift)) | ((error as u64) << shift);
    }

    fn take_error(&mut self, slot: usize) -> Option<PipeError> {
        let shift = slot * EPX_ERROR_BITS;
        let code = (self.error >> shift) & EPX_ERROR_MASK;
        self.error &= !(EPX_ERROR_MASK << shift);
        EpxError::from_code(code).map(PipeError::from)
    }

    /// Void any outcome the interrupt recorded for a pipe's finished attempt.
    fn discard_outcome(&mut self, slot: usize) {
        self.take_completed(slot);
        self.take_error(slot);
        self.take_yielded(slot);
    }
}

/// Stop the in-flight EPX transaction at its next safe boundary so a queued pipe can take
/// the endpoint. Armed only while contended, or an idle bulk IN would trip it on every NAK.
fn arm_epx_yield<T: SealedHostInstance>() {
    #[cfg(feature = "_rp235x")]
    {
        // RP235x stops EPX at its next NAK and raises an interrupt.
        T::regs().nak_poll().modify(|w| w.set_stop_epx_on_nak(true));
        T::regs().inte().modify(|w| w.set_epx_stopped_on_nak(true));
    }
    #[cfg(feature = "rp2040")]
    {
        // RP2040 has no stop-on-NAK bit, so stretch the retry interval and watch frame
        // boundaries instead, as TinyUSB's fallback does.
        T::regs().nak_poll().write(|w| {
            w.set_delay_fs(NAK_POLL_DELAY_YIELD);
            w.set_delay_ls(NAK_POLL_DELAY_YIELD);
        });
        T::regs().inte().modify(|w| w.set_host_sof(true));
    }
}

/// Stop asking for a yield: EPX is ours, or nobody is queued for it.
fn disarm_epx_yield<T: SealedHostInstance>() {
    #[cfg(feature = "_rp235x")]
    {
        T::regs().nak_poll().modify(|w| w.set_stop_epx_on_nak(false));
        T::regs().inte().modify(|w| w.set_epx_stopped_on_nak(false));
    }
    #[cfg(feature = "rp2040")]
    {
        T::host_state().epx.with_arbiter(|a| a.switch_requested = false);
        T::regs().inte().modify(|w| w.set_host_sof(false));
        T::regs().nak_poll().write(|w| {
            w.set_delay_fs(NAK_POLL_DELAY_NORMAL);
            w.set_delay_ls(NAK_POLL_DELAY_NORMAL);
        });
    }
}

#[allow(unused_variables)]
/// Note that EPX made progress, so a pending yield request starts counting again.
pub(super) fn note_epx_progress<T: SealedHostInstance>() {
    #[cfg(feature = "rp2040")]
    T::host_state().epx.with_arbiter(|a| a.switch_requested = false);
}

/// Record a terminal error against the EPX owner and wake it.
pub(super) fn record_epx_error<T: SealedHostInstance>(error: EpxError) {
    let state = T::host_state();

    // With no EPX transaction in flight the error belongs to a polled endpoint. One
    // raised while EPX is also active is still charged here; the source is unknowable.
    if !state.epx.with_arbiter(|a| a.armed) {
        return;
    }

    state
        .epx
        .mark_error(state.epx.current_channel.load(Ordering::Acquire), error);
    state.epx.wake_current();
}

/// Hand EPX from the pipe at `index` to whoever is queued for it. Returns `true` when the
/// transfer was stopped short; a packet landing as the stop takes effect stays with its owner.
fn yield_epx<T: SealedHostInstance>(index: usize) -> bool {
    let state = T::host_state();

    // Sample after the stop. `full` is set by the processor when arming an OUT and by the
    // controller on a received IN, so what counts as "nothing moved" depends on direction.
    let buf_ctrl = T::dpram().ep_in_buffer_control(0);
    let bc = buf_ctrl.read();
    let is_out = T::regs().sie_ctrl().read().send_data();
    let completed = !bc.available(0) && if is_out { !bc.full(0) } else { bc.full(0) };

    if completed {
        // The stop suppressed `trans_complete`, so report it in software and let the
        // owner collect its packet. It keeps EPX until the transfer finishes.
        state.epx.mark_completed(index);
        state.epx.wake_current();
        return false;
    }

    state.epx.mark_yielded(index);

    // Nothing was transferred. Revoke the buffer before another pipe installs its own.
    buf_ctrl.modify(|w| w.set_available(0, false));
    state.epx.current_channel.store(0, Ordering::Release);
    state.epx.wake_all();

    true
}

/// Service the RP235x stop-on-NAK interrupt.
pub(super) fn on_nak_stop<T: SealedHostInstance>() -> bool {
    #[cfg(feature = "_rp235x")]
    {
        let regs = T::regs();
        if regs.ints().read().epx_stopped_on_nak() {
            let index = T::host_state().epx.current_channel.load(Ordering::Acquire);
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

/// Service SOF, using RP2040 frame boundaries to yield a NAKing EPX owner.
pub(super) fn on_host_sof<T: SealedHostInstance>() -> &'static str {
    #[allow(unused)]
    let regs = T::regs();
    #[cfg(feature = "_rp235x")]
    {
        regs.inte().write_clear(|w| w.set_host_sof(true));
        "sof"
    }
    #[cfg(feature = "rp2040")]
    {
        // Reading SOF_RD acknowledges HOST_SOF.
        let _ = regs.sof_rd().read();
        let state = T::host_state();
        let index = state.epx.current_channel.load(Ordering::Acquire);

        if state.epx.with_arbiter(|a| a.waiting) == 0 {
            disarm_epx_yield::<T>();
            "sof idle"
        } else if index == 0 {
            state.epx.with_arbiter(|a| a.switch_requested = false);
            "sof free"
        } else if state.epx.with_arbiter(|a| a.switch_requested) {
            // A second SOF without progress means the holder is retrying NAKs.
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

/// Queue membership for a pipe waiting on EPX. Clearing the bit on drop keeps a
/// cancelled wait from stalling the rotation on a pipe that will never claim it.
struct WaitTicket<T: SealedHostInstance> {
    slot: usize,
    _phantom: PhantomData<T>,
}

impl<T: SealedHostInstance> WaitTicket<T> {
    fn new(slot: usize) -> Self {
        T::host_state().epx.with_arbiter(|a| a.set_waiting(slot, true));

        Self {
            slot,
            _phantom: PhantomData,
        }
    }
}

impl<T: SealedHostInstance> Drop for WaitTicket<T> {
    fn drop(&mut self) {
        T::host_state().epx.with_arbiter(|a| a.set_waiting(self.slot, false));
    }
}

/// Number of [`EPX_BLOCK_SIZE`] blocks needed to hold `len` bytes.
fn blocks_for(len: u16) -> usize {
    (len as usize).div_ceil(EPX_BLOCK_SIZE)
}

/// `used_blocks` mask for a run of `blocks` blocks starting at `start`.
fn block_mask(start: usize, blocks: usize) -> u64 {
    ((1u64 << blocks) - 1) << start
}

/// Allocate `len` bytes of EPX buffer memory, returning its [`super::super::EP_MEMORY`]-relative byte address.
/// First-fit over contiguous free blocks.
fn alloc_epx_mem(state: &EpxState, len: u16) -> Result<u16, ()> {
    let blocks = blocks_for(len);
    if blocks == 0 || blocks > EPX_NUM_BLOCKS {
        error!("EPX buffer request of {} bytes is too large", len);
        return Err(());
    }
    critical_section::with(|cs| {
        let used_blocks = state.used_blocks.borrow(cs);
        let used = used_blocks.get();
        for start in 0..=(EPX_NUM_BLOCKS - blocks) {
            let mask = block_mask(start, blocks);
            if used & mask == 0 {
                used_blocks.set(used | mask);
                return Ok(EPX_BUFFER_OFFSET + (start * EPX_BLOCK_SIZE) as u16);
            }
        }
        error!("EPX buffer memory full");
        Err(())
    })
}

/// Free EPX buffer memory previously returned by [`alloc_epx_mem`].
fn free_epx_mem(state: &EpxState, addr: u16, len: u16) {
    let blocks = blocks_for(len);
    let start = (addr - EPX_BUFFER_OFFSET) as usize / EPX_BLOCK_SIZE;
    let mask = block_mask(start, blocks);
    critical_section::with(|cs| {
        let used_blocks = state.used_blocks.borrow(cs);
        used_blocks.set(used_blocks.get() & !mask);
    });
}

/// EPX arbitration state.
///
/// RP2040 gives non-interrupt transfers one shared endpoint, so every control, bulk and
/// isochronous pipe queues for it. A controller with an endpoint per pipe needs none of it.
pub(super) struct EpxState {
    /// Current channel with ongoing non-interrupt transfer. `0` means None.
    current_channel: AtomicUsize,
    /// Bitset of allocated EPX pipes, indexed by `Channel::index - EP_COUNT`.
    allocated_slots: AtomicU16,
    /// One waiter per EPX pipe. Completion is routed to the pipe that owns EPX,
    /// while releasing it wakes every contender.
    wakers: [AtomicWaker; EPX_MAX_PIPES],
    /// Bitmap of used EPX buffer blocks, one bit per [`EPX_BLOCK_SIZE`] block.
    used_blocks: critical_section::Mutex<Cell<u64>>,
    /// One lock keeps multi-field decisions consistent; it also masks the interrupt, so a
    /// re-entrant borrow cannot happen.
    arbiter: critical_section::Mutex<RefCell<EpxArbiter>>,
}

impl EpxState {
    /// Run `f` with the arbiter locked.
    fn with_arbiter<R>(&self, f: impl FnOnce(&mut EpxArbiter) -> R) -> R {
        critical_section::with(|cs| f(&mut self.arbiter.borrow(cs).borrow_mut()))
    }

    pub(super) const fn new() -> Self {
        Self {
            current_channel: AtomicUsize::new(0),
            allocated_slots: AtomicU16::new(0),
            wakers: [const { AtomicWaker::new() }; EPX_MAX_PIPES],
            used_blocks: critical_section::Mutex::new(Cell::new(0)),
            arbiter: critical_section::Mutex::new(RefCell::new(EpxArbiter::new())),
        }
    }

    pub(super) fn reset(&self) {
        self.current_channel.store(0, Ordering::Relaxed);
        self.allocated_slots.store(0, Ordering::Relaxed);
        critical_section::with(|cs| {
            *self.arbiter.borrow(cs).borrow_mut() = EpxArbiter::new();
            self.used_blocks.borrow(cs).set(0);
        });
    }

    pub(super) fn is_idle(&self) -> bool {
        self.current_channel.load(Ordering::Acquire) == 0 && !self.with_arbiter(|arbiter| arbiter.armed)
    }

    pub(super) fn allocate(&self, max_packet_size: u16) -> Result<(usize, u16, u16), HostError> {
        let slot = critical_section::with(|_| {
            let allocated = self.allocated_slots.load(Ordering::Relaxed);
            let slot = allocated.trailing_ones() as usize;
            if slot >= EPX_MAX_PIPES {
                return Err(HostError::OutOfPipes);
            }
            self.allocated_slots.store(allocated | (1 << slot), Ordering::Relaxed);
            Ok(slot)
        })?;

        // A parked transfer keeps its buffer while another pipe owns EPX.
        let len = (blocks_for(max_packet_size) * EPX_BLOCK_SIZE) as u16;
        let addr = match alloc_epx_mem(self, len) {
            Ok(addr) => addr,
            Err(()) => {
                // Hand the slot back or repeated buffer failures exhaust the pipe pool.
                critical_section::with(|_| {
                    let allocated = self.allocated_slots.load(Ordering::Relaxed);
                    self.allocated_slots.store(allocated & !(1 << slot), Ordering::Relaxed);
                });
                return Err(HostError::InsufficientMemory);
            }
        };

        debug!(
            "EPX pipe ALLOC slot {} (bitset {:04x}) buf {:#x}+{}",
            slot,
            self.allocated_slots.load(Ordering::Relaxed),
            addr,
            len
        );
        Ok((EP_COUNT + slot, addr, len))
    }

    pub(super) fn free(&self, slot: usize, addr: u16, len: u16) {
        debug_assert!(slot < EPX_MAX_PIPES);
        free_epx_mem(self, addr, len);
        self.discard_outcome(slot);
        critical_section::with(|_| {
            let allocated = self.allocated_slots.load(Ordering::Relaxed);
            self.allocated_slots.store(allocated & !(1 << slot), Ordering::Relaxed);
        });
        debug!(
            "EPX pipe FREE  slot {} (bitset {:04x})",
            slot,
            self.allocated_slots.load(Ordering::Relaxed)
        );
    }

    /// Slot of an EPX pipe in [`EpxState::wakers`], or `None` for the idle
    /// sentinel and for interrupt pipes.
    fn slot(index: usize) -> Option<usize> {
        index.checked_sub(EP_COUNT).filter(|slot| *slot < EPX_MAX_PIPES)
    }

    /// Wake the pipe currently holding EPX.
    fn wake_current(&self) {
        if let Some(slot) = Self::slot(self.current_channel.load(Ordering::Acquire)) {
            self.wakers[slot].wake();
        }
    }

    /// Wake every allocated EPX pipe, so whoever is queued can claim EPX.
    fn wake_all(&self) {
        let allocated = self.allocated_slots.load(Ordering::Acquire);
        for slot in 0..EPX_MAX_PIPES {
            if allocated & (1 << slot) != 0 {
                self.wakers[slot].wake();
            }
        }
    }

    /// Slot whose turn it is to take a free EPX. `None` when nobody is queued.
    fn turn(&self) -> Option<usize> {
        self.with_arbiter(|a| a.turn())
    }

    /// Record that the pipe holding EPX had its transaction stopped.
    fn mark_yielded(&self, index: usize) {
        if let Some(slot) = Self::slot(index) {
            self.with_arbiter(|a| a.mark_yielded(slot));
        }
    }

    /// Record a completion the hardware could not signal because EPX was stopped.
    fn mark_completed(&self, index: usize) {
        if let Some(slot) = Self::slot(index) {
            self.with_arbiter(|a| a.mark_completed(slot));
        }
    }

    /// Clear a pipe's completion bit, returning whether it was set.
    fn take_completed(&self, slot: usize) -> bool {
        self.with_arbiter(|a| a.take_completed(slot))
    }

    /// Record a terminal error against the pipe currently holding EPX.
    fn mark_error(&self, index: usize, error: EpxError) {
        if let Some(slot) = Self::slot(index) {
            self.with_arbiter(|a| a.mark_error(slot, error));
        }
    }

    /// Take any terminal error recorded for a pipe.
    fn take_error(&self, slot: usize) -> Option<PipeError> {
        self.with_arbiter(|a| a.take_error(slot))
    }

    /// Clear a pipe's yield bit, returning whether it was set.
    fn take_yielded(&self, slot: usize) -> bool {
        self.with_arbiter(|a| a.take_yielded(slot))
    }

    /// Void any outcome the interrupt recorded for a pipe's finished attempt.
    fn discard_outcome(&self, slot: usize) {
        self.with_arbiter(|a| a.discard_outcome(slot));
    }
}

pub(super) fn wake_current<T: SealedHostInstance>() {
    T::host_state().epx.wake_current();
}

enum TransactionStatus {
    Complete,
    NakYield,
    Timeout,
}

struct EpxTransactionGuard<T: SealedHostInstance> {
    state: &'static HostState,
    index: usize,
    ep_control: EpControlReg,
    buffer_control: BufferControlReg,
    transaction_active: bool,
    _phantom: PhantomData<T>,
}

impl<T: SealedHostInstance> EpxTransactionGuard<T> {
    fn new<'d, E: pipe::Type, D: pipe::Direction>(channel: &Channel<'d, T, E, D>) -> Self {
        debug_assert!(E::ep_type() != EndpointType::Interrupt);

        Self {
            state: T::host_state(),
            index: channel.index,
            ep_control: channel.ep_control(),
            buffer_control: channel.buffer_control(),
            transaction_active: false,
            _phantom: PhantomData,
        }
    }

    fn arm(&mut self) {
        self.transaction_active = true;
        self.state.epx.with_arbiter(|a| a.armed = true);
    }

    fn disarm(&mut self) {
        self.transaction_active = false;
        self.state.epx.with_arbiter(|a| a.armed = false);
    }
}

impl<T: SealedHostInstance> Drop for EpxTransactionGuard<T> {
    fn drop(&mut self) {
        let selected = self.state.epx.current_channel.load(Ordering::Relaxed);
        if selected == self.index || selected == 0 {
            // Stop the SIE before reconfiguring EPX.
            if self.transaction_active {
                let regs = T::regs();
                regs.sie_ctrl().modify(|w| w.set_stop_trans(true));
                while regs.sie_ctrl().read().stop_trans() {}
                regs.sie_status().write_clear(|w| {
                    w.set_trans_complete(true);
                    w.set_stall_rec(true);
                    w.set_rx_timeout(true);
                    w.set_rx_overflow(true);
                });
                regs.buff_status().write_clear(|w| w.0 = 0b11);
            }
            if let Some(slot) = EpxState::slot(self.index) {
                self.state.epx.with_arbiter(|a| a.last = slot);

                // Do not leak a cancelled attempt's outcome.
                self.state.epx.discard_outcome(slot);
            }
            self.state.epx.with_arbiter(|a| a.armed = false);
            self.state.epx.current_channel.store(0, Ordering::Release);
            disarm_epx_yield::<T>();
            self.state.epx.wake_all();

            self.ep_control.modify(|w| {
                w.set_interrupt_per_buff(false);
                w.set_enable(false);
            });
            self.buffer_control.modify(|w| w.set_available(0, false));
        }
    }
}

impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> Channel<'d, T, E, D> {
    fn take_nak_yield(&self) -> bool {
        T::host_state().epx.take_yielded(self.epx_slot())
    }

    fn waker(&self) -> &AtomicWaker {
        &T::host_state().epx.wakers[self.epx_slot()]
    }

    /// Wait until EPX is selected for this pipe and its buffer is idle.
    async fn wait_epx_buffer(&self) {
        trace!("CHANNEL {} WAIT AVAILABLE", self.index);
        poll_fn(|cx| {
            self.waker().register(cx.waker());

            let reg = self.buffer_control().read();
            if self.can_run_transaction() {
                T::regs().buff_status().write_clear(|w| w.0 = 0b11);
            }

            if reg.available(0) {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await
    }

    /// Configure the shared endpoint for the selected pipe.
    fn configure_epx_for_pipe(&self) {
        trace!(
            "SET CURRENT: {:?} CHANNEL {}: dev: {}, ep: {}, max_packet: {}, preamble: {}",
            E::ep_type(),
            self.index,
            self.dev_addr,
            self.ep_addr,
            self.max_packet_size,
            self.pre
        );
        T::host_state().epx.current_channel.store(self.index, Ordering::Relaxed);

        T::regs().addr_endp().write(|w| {
            w.set_address(self.dev_addr);
            w.set_endpoint(self.ep_addr);
        });

        self.ep_control().modify(|w| {
            w.set_enable(true);
            w.set_interrupt_per_buff(true);
            w.set_buffer_address(self.buf.addr);
            w.set_endpoint_type(match E::ep_type() {
                EndpointType::Control => EpControlEndpointType::Control,
                EndpointType::Isochronous => EpControlEndpointType::Isochronous,
                EndpointType::Bulk => EpControlEndpointType::Bulk,
                EndpointType::Interrupt => EpControlEndpointType::Interrupt,
            });
        });

        T::regs().sie_ctrl().modify(|w| w.set_preamble_en(self.pre));
    }

    fn stop_timed_out_transaction(&self) -> TransactionStatus {
        let regs = T::regs();
        let setup = regs.sie_ctrl().read().send_setup();
        regs.sie_ctrl().modify(|w| w.set_stop_trans(true));
        while regs.sie_ctrl().read().stop_trans() {}

        if setup && regs.sie_status().read().trans_complete() {
            regs.sie_status().write_clear(|w| w.set_trans_complete(true));
            TransactionStatus::Complete
        } else if !setup && !yield_epx::<T>(self.index) {
            T::host_state().epx.take_completed(self.epx_slot());
            TransactionStatus::Complete
        } else {
            // Do not expose this result to the next owner.
            regs.sie_status().write_clear(|w| {
                w.set_trans_complete(true);
                w.set_stall_rec(true);
                w.set_rx_timeout(true);
                w.set_rx_overflow(true);
            });
            regs.buff_status().write_clear(|w| w.0 = 0b11);
            TransactionStatus::Timeout
        }
    }

    /// Do we hold the shared endpoint, or may we take it next?
    fn can_run_transaction(&self) -> bool {
        let state = T::host_state();
        let sel = state.epx.current_channel.load(Ordering::Relaxed);
        if sel == self.index {
            return true;
        }
        if sel != 0 {
            return false;
        }

        // EPX is free: take it only when the queue says it is our turn, so a
        // pipe that keeps re-arming cannot monopolise the endpoint.
        match state.epx.turn() {
            Some(slot) => slot == self.epx_slot(),
            None => true,
        }
    }

    /// Take EPX for this pipe. The guard releases it.
    ///
    /// The ticket must span both waits: leaving the queue early could move the turn
    /// while this pipe is still settling its buffer.
    async fn acquire_epx(&self) -> EpxTransactionGuard<T> {
        trace!("CHANNEL {} WAIT READY", self.index);

        {
            let _ticket = WaitTicket::<T>::new(self.epx_slot());
            poll_fn(|cx| {
                self.waker().register(cx.waker());

                if self.can_run_transaction() {
                    #[cfg(feature = "_rp235x")]
                    disarm_epx_yield::<T>();

                    return Poll::Ready(());
                }

                trace!("CHANNEL {} EPX contention: request yield", self.index);
                arm_epx_yield::<T>();

                Poll::Pending
            })
            .await;

            self.wait_epx_buffer().await;
        }

        self.configure_epx_for_pipe();
        EpxTransactionGuard::new(self)
    }

    // FIXME: RX Timeout with LS device on hub
    /// Start transaction and wait it to be complete
    async fn wait_epx_transaction(&self) -> Result<TransactionStatus, PipeError> {
        assert!(E::ep_type() != EndpointType::Interrupt);
        let regs = T::regs();

        regs.inte().modify(|w| {
            w.set_trans_complete(true);
            w.set_stall(true);
            // Unreliable for low-speed devices behind hubs.
            w.set_error_rx_timeout(false);
            w.set_error_rx_overflow(true);
            w.set_error_crc(true);
            w.set_error_bit_stuff(true);
            w.set_error_data_seq(true);
        });

        // START_TRANS is synchronized separately (RP2040 §4.1.2.9, RP2350 §12.7.3.9).
        cortex_m::asm::delay(SIE_START_DELAY_CYCLES);
        T::regs().sie_ctrl().modify(|w| {
            w.set_start_trans(true);
        });

        trace!("CHANNEL {} WAIT TRANSACTION", self.index);
        let res = poll_fn(|cx| {
            self.waker().register(cx.waker());

            if let Some(error) = T::host_state().epx.take_error(self.epx_slot()) {
                return Poll::Ready(Err(error));
            }
            if T::host_state().epx.take_completed(self.epx_slot()) {
                return Poll::Ready(Ok(TransactionStatus::Complete));
            }
            if self.take_nak_yield() {
                return Poll::Ready(Ok(TransactionStatus::NakYield));
            }

            let stat = regs.sie_status().read();
            if stat.trans_complete() {
                regs.sie_status().write_clear(|w| w.set_trans_complete(true));
                return Poll::Ready(Ok(TransactionStatus::Complete));
            }
            if stat.stall_rec() {
                regs.sie_status().write_clear(|w| w.set_stall_rec(true));
                return Poll::Ready(Err(PipeError::Stall));
            }
            if stat.rx_overflow() {
                regs.sie_status().write_clear(|w| w.set_rx_overflow(true));
                return Poll::Ready(Err(PipeError::BufferOverflow));
            }
            Poll::Pending
        })
        .await;

        res
    }

    /// Copy setup packet to buffer and set SETUP transaction
    ///
    /// Set PID = 1 for next transaction
    pub(super) fn set_setup_packet(&mut self, setup: &[u8; 8]) {
        assert!(E::ep_type() == EndpointType::Control);
        let dpram = T::dpram();
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        let index = u16::from_le_bytes([setup[4], setup[5]]);
        let length = u16::from_le_bytes([setup[6], setup[7]]);
        dpram.setup_packet_low().write(|w| {
            w.set_bmrequesttype(setup[0]);
            w.set_brequest(setup[1]);
            w.set_wvalue(value);
        });
        dpram.setup_packet_high().write(|w| {
            w.set_windex(index);
            w.set_wlength(length);
        });
        T::regs().sie_ctrl().modify(|w| {
            w.set_send_data(false);
            w.set_receive_data(false);
            w.set_send_setup(true);
        });
    }

    /// Set DATA IN transaction
    ///
    pub(super) fn set_data_in(&mut self, len: u16, pid: bool) {
        assert!(E::ep_type() != EndpointType::Interrupt);
        let pid = if E::ep_type() == EndpointType::Isochronous {
            false
        } else {
            pid
        };

        self.write_buffer_control(|w| {
            w.set_pid(0, pid);
            w.set_full(0, false);
            w.set_length(0, len);
            w.set_last(0, true);
            w.set_reset(true);
            w.set_available(0, true);
        });

        T::regs().sie_ctrl().modify(|w| {
            w.set_send_data(false);
            w.set_send_setup(false);
            w.set_receive_data(true);
        });
    }

    /// Set DATA OUT transaction and copy data to buffer
    /// Returns count of copied bytes
    pub(super) fn set_data_out(&mut self, data: &[u8], pid: bool) -> usize {
        assert!(E::ep_type() != EndpointType::Interrupt);
        let pid = if E::ep_type() == EndpointType::Isochronous {
            false
        } else {
            pid
        };

        let chunk = if data.len() > 0 {
            data.chunks(self.max_packet_size as _).next().unwrap()
        } else {
            &[]
        };

        self.buf.write(&chunk);

        self.write_buffer_control(|w| {
            w.set_available(0, true);
            w.set_pid(0, pid);
            w.set_full(0, true);
            w.set_length(0, chunk.len() as _);
            w.set_last(0, true);
            w.set_reset(true);
        });

        T::regs().sie_ctrl().modify(|w| {
            w.set_send_data(true);
            w.set_send_setup(false);
            w.set_receive_data(false);
        });

        chunk.len()
    }

    /// Claim EPX and run one transaction on it. `arm` programs the buffer once this
    /// pipe owns the endpoint.
    pub(super) async fn run_epx_transaction(&mut self, mut arm: impl FnMut(&mut Self)) -> Result<(), PipeError> {
        let deadline = self
            .control_timeout_us
            .map(|us| embassy_time::Instant::now() + embassy_time::Duration::from_micros(us));

        loop {
            let mut guard = self.acquire_epx().await;

            arm(self);
            guard.arm();
            let result = if let Some(deadline) = deadline {
                match embassy_time::with_deadline(deadline, self.wait_epx_transaction()).await {
                    Ok(result) => result,
                    Err(_) => Ok(self.stop_timed_out_transaction()),
                }
            } else {
                self.wait_epx_transaction().await
            };

            // Only completion proves that the SIE released EPX. Other outcomes leave
            // the guard armed so Drop stops the transaction.
            match result? {
                TransactionStatus::Complete => {
                    guard.disarm();
                    return Ok(());
                }
                // Isochronous data cannot be replayed in a later frame.
                TransactionStatus::NakYield if E::ep_type() == EndpointType::Isochronous => {
                    return Err(PipeError::Canceled);
                }
                TransactionStatus::NakYield => {}
                TransactionStatus::Timeout => {
                    guard.disarm();
                    return Err(PipeError::Timeout);
                }
            }

            drop(guard);
            embassy_futures::yield_now().await;
        }
    }

    /// Receive one packet, returning its length.
    pub(super) async fn transfer_in_packet(&mut self, len: u16, pid: bool) -> Result<usize, PipeError> {
        self.run_epx_transaction(|s| s.set_data_in(len, pid)).await?;

        Ok(self.buffer_control().read().length(0) as usize)
    }

    /// Send one packet, returning how much of `data` it carried.
    pub(super) async fn transfer_out_packet(&mut self, data: &[u8], pid: bool) -> Result<usize, PipeError> {
        let mut len = 0;
        self.run_epx_transaction(|s| len = s.set_data_out(data, pid)).await?;

        Ok(len)
    }
}

impl<'d, T: SealedHostInstance, E, D> Channel<'d, T, E, D> {
    /// This pipe's slot in the EPX arrays. Only meaningful for non-interrupt pipes.
    pub(super) fn epx_slot(&self) -> usize {
        self.index - EP_COUNT
    }
}
