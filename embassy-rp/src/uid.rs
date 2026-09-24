//! Unique ID (UID)
//!
//! The ID is read once, on first use, and cached. Read it during startup, before core 1 is
//! spawned and before any other flash access on RP2040, where obtaining it runs a command on
//! the flash chip with interrupts disabled and core 1 paused.
//!
//! The source differs per chip:
//!
//! - **RP2350**: the chip ID held in OTP, which is also what the bootrom reports as its USB
//!   serial number.
//! - **RP2040**: the die has no unique ID, so the SPI flash chip's unique ID is used instead,
//!   matching `pico_get_unique_board_id` in the pico-sdk. Not every flash chip implements the
//!   command, and the bytes are predictable enough that they should be salted and hashed
//!   before being used for anything like a MAC address.
//!
//! If the ID cannot be read, every byte reads back as `0xEE`, following the pico-sdk's
//! convention for a well-defined and obviously wrong value.

use once_cell::sync::Lazy;

/// Value reported for every byte when the ID cannot be read.
const UNAVAILABLE: u8 = 0xEE;

struct Uid {
    bytes: [u8; 8],
    hex: [u8; 16],
}

static UID: Lazy<Uid> = Lazy::new(|| {
    let bytes = read_uid();

    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut hex = [0u8; 16];
    for (idx, v) in bytes.iter().enumerate() {
        let lo = v & 0x0f;
        let hi = (v & 0xf0) >> 4;
        hex[idx * 2] = HEX[hi as usize];
        hex[idx * 2 + 1] = HEX[lo as usize];
    }

    Uid { bytes, hex }
});

#[cfg(feature = "rp2040")]
fn read_uid() -> [u8; 8] {
    let mut bytes = [0u8; 8];
    // Does not take the FLASH peripheral: `in_ram` already pauses core 1 and runs the command
    // in a critical section, and the ID is read-only.
    match unsafe { crate::flash::in_ram(|| crate::flash::ram_helpers::flash_unique_id(&mut bytes)) } {
        Ok(()) => bytes,
        Err(_) => [UNAVAILABLE; 8],
    }
}

#[cfg(feature = "_rp235x")]
fn read_uid() -> [u8; 8] {
    match crate::otp::get_chipid() {
        Ok(id) => id.to_be_bytes(),
        Err(_) => [UNAVAILABLE; 8],
    }
}

/// Get this device's unique 64-bit ID.
pub fn uid() -> &'static [u8; 8] {
    &UID.bytes
}

/// Get this device's unique 64-bit ID, encoded into a string of 16 hexadecimal ASCII digits.
pub fn uid_hex() -> &'static str {
    unsafe { core::str::from_utf8_unchecked(uid_hex_bytes()) }
}

/// Get this device's unique 64-bit ID, encoded into 16 hexadecimal ASCII bytes.
pub fn uid_hex_bytes() -> &'static [u8; 16] {
    &UID.hex
}
