//! EPX: the single shared endpoint RP2040 gives non-interrupt transfers.
//!
//! Control, bulk and isochronous pipes all queue for one endpoint, so they need a turn
//! order, a buffer allocator and a preemption scheme. Interrupt endpoints need none of it,
//! and neither would a controller with an endpoint per pipe.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_usb_driver::EndpointType;
use embassy_usb_driver::host::{PipeError, pipe};

use super::super::{DPRAM_DATA_OFFSET, EP_COUNT, EP_MEMORY_SIZE};
use super::{Channel, EpxTransactionGuard, HostState, SIE_START_DELAY_CYCLES, SealedHostInstance, TransactionStatus};
use crate::RegExt;

// DPRAM layout - `EP_COUNT` blocks for control and interrupts, then remaining blocks
// allocatable to EPX pipes.
pub(super) const EPX_BLOCK_SIZE: usize = 64;

/// First EPX buffer after the control and interrupt buffers.
pub(super) const EPX_BUFFER_OFFSET: u16 = DPRAM_DATA_OFFSET + ((EP_COUNT - 1) * EPX_BLOCK_SIZE) as u16;

/// Number of allocatable EPX blocks.
pub(super) const EPX_NUM_BLOCKS: usize = (EP_MEMORY_SIZE - EPX_BUFFER_OFFSET as usize) / EPX_BLOCK_SIZE;

/// Maximum number of concurrently allocated EPX pipes.
pub(super) const EPX_MAX_PIPES: usize = 16;

/// NAK retry interval, stretched so a NAKing transfer cannot retry within the frame.
#[cfg(feature = "rp2040")]
const NAK_POLL_DELAY_YIELD: u16 = 300;

/// NAK retry interval outside a yield.
#[cfg(feature = "rp2040")]
const NAK_POLL_DELAY_NORMAL: u16 = 16;

/// Bits reserved per EPX pipe in [`EpxArbiter::error`].
pub(super) const EPX_ERROR_BITS: usize = 4;

/// Mask of one pipe's error field.
pub(super) const EPX_ERROR_MASK: u64 = (1 << EPX_ERROR_BITS) - 1;

/// Terminal errors the interrupt handler can record, packed [`EPX_ERROR_BITS`] wide.
/// Zero means no error, so the whole field clears to "nothing recorded".
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u64)]
pub(super) enum EpxError {
    BadResponse = 1,
    DataToggle = 2,
}

impl EpxError {
    pub(super) fn from_code(code: u64) -> Option<Self> {
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
pub(super) struct EpxArbiter {
    /// Bitset of EPX pipes whose transaction was stopped at a NAK boundary.
    yielded: u16,
    /// Bitset of EPX pipes queued in `lock_epx`.
    pub(super) waiting: u16,
    /// Whether EPX has a transaction armed. Framing errors come from the shared RX
    /// engine, so they are only charged to EPX while it is mid-transaction.
    pub(super) armed: bool,
    /// Bitset of EPX pipes whose packet landed as the transfer was being stopped.
    /// `STOP_TRANS` suppresses `trans_complete`, so the completion is reported here.
    completed: u16,
    /// Terminal error per EPX pipe, [`EPX_ERROR_BITS`] wide. Clearing the status bit is
    /// what stops the interrupt re-firing, so the reason is carried here instead.
    error: u64,
    /// Slot that last held EPX. The queue is served starting after it, so a pipe
    /// that just released goes to the back.
    pub(super) last: usize,
    /// RP2040 only: set on the first SOF of a contended EPX transfer, so the
    /// second SOF with no completion in between can stop it.
    #[cfg(feature = "rp2040")]
    pub(super) switch_requested: bool,
}

impl EpxArbiter {
    /// Arbitration state with nothing queued, held or recorded.
    pub(super) const fn new() -> Self {
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
    pub(super) fn turn(&self) -> Option<usize> {
        if self.waiting == 0 {
            return None;
        }
        (1..=EPX_MAX_PIPES)
            .map(|step| (self.last + step) % EPX_MAX_PIPES)
            .find(|slot| self.waiting & (1 << slot) != 0)
    }

    /// Queue a pipe for EPX, or take it out of the queue.
    pub(super) fn set_waiting(&mut self, slot: usize, waiting: bool) {
        if waiting {
            self.waiting |= 1 << slot;
        } else {
            self.waiting &= !(1 << slot);
        }
    }

    pub(super) fn mark_yielded(&mut self, slot: usize) {
        self.yielded |= 1 << slot;
    }

    pub(super) fn take_yielded(&mut self, slot: usize) -> bool {
        let was = self.yielded & (1 << slot) != 0;
        self.yielded &= !(1 << slot);
        was
    }

    pub(super) fn mark_completed(&mut self, slot: usize) {
        self.completed |= 1 << slot;
    }

    pub(super) fn take_completed(&mut self, slot: usize) -> bool {
        let was = self.completed & (1 << slot) != 0;
        self.completed &= !(1 << slot);
        was
    }

    pub(super) fn mark_error(&mut self, slot: usize, error: EpxError) {
        let shift = slot * EPX_ERROR_BITS;
        self.error = (self.error & !(EPX_ERROR_MASK << shift)) | ((error as u64) << shift);
    }

    pub(super) fn take_error(&mut self, slot: usize) -> Option<PipeError> {
        let shift = slot * EPX_ERROR_BITS;
        let code = (self.error >> shift) & EPX_ERROR_MASK;
        self.error &= !(EPX_ERROR_MASK << shift);
        EpxError::from_code(code).map(PipeError::from)
    }

    /// Void any outcome the interrupt recorded for a pipe's finished attempt.
    pub(super) fn discard_outcome(&mut self, slot: usize) {
        self.take_completed(slot);
        self.take_error(slot);
        self.take_yielded(slot);
    }
}

/// Stop the in-flight EPX transaction at its next safe boundary so a queued pipe can take
/// the endpoint. Armed only while contended, or an idle bulk IN would trip it on every NAK.
pub(super) fn arm_epx_yield<T: SealedHostInstance>() {
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
pub(super) fn disarm_epx_yield<T: SealedHostInstance>() {
    #[cfg(feature = "_rp235x")]
    {
        T::regs().nak_poll().modify(|w| w.set_stop_epx_on_nak(false));
        T::regs().inte().modify(|w| w.set_epx_stopped_on_nak(false));
    }
    #[cfg(feature = "rp2040")]
    {
        T::host_state().with_arbiter(|a| a.switch_requested = false);
        T::regs().inte().modify(|w| w.set_host_sof(false));
        T::regs().nak_poll().write(|w| {
            w.set_delay_fs(NAK_POLL_DELAY_NORMAL);
            w.set_delay_ls(NAK_POLL_DELAY_NORMAL);
        });
    }
}

/// Note that EPX made progress, so a pending yield request starts counting again.
#[allow(unused_variables)]
pub(super) fn note_epx_progress<T: SealedHostInstance>() {
    #[cfg(feature = "rp2040")]
    T::host_state().with_arbiter(|a| a.switch_requested = false);
}

/// Record a terminal error against the EPX owner and wake it.
pub(super) fn record_epx_error<T: SealedHostInstance>(error: EpxError) {
    let state = T::host_state();

    // With no EPX transaction in flight the error belongs to a polled endpoint. One
    // raised while EPX is also active is still charged here; the source is unknowable.
    if !state.with_arbiter(|a| a.armed) {
        return;
    }

    state.mark_epx_error(state.current_channel.load(Ordering::Acquire), error);
    state.wake_current_epx();
}

/// Hand EPX from the pipe at `index` to whoever is queued for it. Returns `true` when the
/// transfer was stopped short; a packet landing as the stop takes effect stays with its owner.
pub(super) fn yield_epx<T: SealedHostInstance>(index: usize) -> bool {
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
        state.mark_epx_completed(index);
        state.wake_current_epx();
        return false;
    }

    state.mark_epx_yielded(index);

    // Nothing was transferred. Revoke the buffer before another pipe installs its own.
    buf_ctrl.modify(|w| w.set_available(0, false));
    state.current_channel.store(0, Ordering::Release);
    state.wake_all_epx();

    true
}

/// Queue membership for a pipe waiting on EPX. Clearing the bit on drop keeps a
/// cancelled wait from stalling the rotation on a pipe that will never claim it.
pub(super) struct WaitTicket<T: SealedHostInstance> {
    slot: usize,
    _phantom: PhantomData<T>,
}

impl<T: SealedHostInstance> WaitTicket<T> {
    pub(super) fn new(slot: usize) -> Self {
        T::host_state().with_arbiter(|a| a.set_waiting(slot, true));

        Self {
            slot,
            _phantom: PhantomData,
        }
    }
}

impl<T: SealedHostInstance> Drop for WaitTicket<T> {
    fn drop(&mut self) {
        T::host_state().with_arbiter(|a| a.set_waiting(self.slot, false));
    }
}

/// Number of [`EPX_BLOCK_SIZE`] blocks needed to hold `len` bytes.
pub(super) fn blocks_for(len: u16) -> usize {
    (len as usize).div_ceil(EPX_BLOCK_SIZE)
}

/// `used_blocks` mask for a run of `blocks` blocks starting at `start`.
pub(super) fn block_mask(start: usize, blocks: usize) -> u64 {
    ((1u64 << blocks) - 1) << start
}

/// Allocate `len` bytes of EPX buffer memory, returning its [`EP_MEMORY`]-relative byte address.
/// First-fit over contiguous free blocks.
pub(super) fn alloc_epx_mem(state: &HostState, len: u16) -> Result<u16, ()> {
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
pub(super) fn free_epx_mem(state: &HostState, addr: u16, len: u16) {
    let blocks = blocks_for(len);
    let start = (addr - EPX_BUFFER_OFFSET) as usize / EPX_BLOCK_SIZE;
    let mask = block_mask(start, blocks);
    critical_section::with(|cs| {
        let used_blocks = state.used_blocks.borrow(cs);
        used_blocks.set(used_blocks.get() & !mask);
    });
}

/// EPX transfers: everything that drives the shared endpoint for a pipe.
impl<'d, T: SealedHostInstance, E: pipe::Type, D: pipe::Direction> Channel<'d, T, E, D> {
    fn stop_timed_out_transaction(&self) -> TransactionStatus {
        let regs = T::regs();
        let setup = regs.sie_ctrl().read().send_setup();
        regs.sie_ctrl().modify(|w| w.set_stop_trans(true));
        while regs.sie_ctrl().read().stop_trans() {}

        if setup && regs.sie_status().read().trans_complete() {
            regs.sie_status().write_clear(|w| w.set_trans_complete(true));
            TransactionStatus::Complete
        } else if !setup && !yield_epx::<T>(self.index) {
            T::host_state().take_epx_completed(self.epx_slot());
            TransactionStatus::Complete
        } else {
            // Clear global status so the next pipe cannot consume this transaction's result.
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
    pub(super) fn can_run_epx_transaction(&self) -> bool {
        let state = T::host_state();
        let sel = state.current_channel.load(Ordering::Relaxed);
        if sel == self.index {
            return true;
        }
        if sel != 0 {
            return false;
        }

        // EPX is free: take it only when the queue says it is our turn, so a
        // pipe that keeps re-arming cannot monopolise the endpoint.
        match state.epx_turn() {
            Some(slot) => slot == self.epx_slot(),
            None => true,
        }
    }

    /// Take EPX for this pipe. The guard releases it.
    ///
    /// The ticket must span both waits: leaving the queue early would let another pipe be
    /// granted the turn while this one is still settling its buffer.
    async fn lock_epx(&self) -> EpxTransactionGuard<T> {
        trace!("CHANNEL {} WAIT READY", self.index);

        {
            let _ticket = WaitTicket::<T>::new(self.epx_slot());
            poll_fn(|cx| {
                self.waker().register(cx.waker());

                if self.can_run_epx_transaction() {
                    #[cfg(feature = "_rp235x")]
                    disarm_epx_yield::<T>();

                    return Poll::Ready(());
                }

                trace!("CHANNEL {} EPX contention: request yield", self.index);
                arm_epx_yield::<T>();

                Poll::Pending
            })
            .await;

            self.wait_available().await;
        }

        self.set_current();
        self.epx_transaction_guard()
    }

    // FIXME: RX Timeout with LS device on hub
    /// Start transaction and wait it to be complete
    async fn wait_epx_transaction(&self) -> Result<TransactionStatus, PipeError> {
        assert!(!Self::is_interrupt());
        let regs = T::regs();

        // Enable error and cplt interrupts
        regs.inte().modify(|w| {
            w.set_trans_complete(true);
            w.set_stall(true);
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

            if let Some(error) = T::host_state().take_epx_error(self.epx_slot()) {
                return Poll::Ready(Err(error));
            }
            if T::host_state().take_epx_completed(self.epx_slot()) {
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
            let mut guard = self.lock_epx().await;

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

            // Only a completed transaction is known to have released the SIE, so any other
            // outcome leaves the guard armed and its `Drop` issues STOP_TRANS.
            match result? {
                TransactionStatus::Complete => {
                    guard.disarm();
                    return Ok(());
                }
                // Isochronous data is only valid in its own frame, so a yielded
                // transfer is dropped rather than replayed later.
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
