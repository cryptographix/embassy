//! Universal MIDI Packet transport support for USB MIDI 2.0.

/// Size of one Universal MIDI Packet word in bytes.
pub const UMP_WORD_SIZE: usize = 4;

/// Maximum number of words in one Universal MIDI Packet.
pub const UMP_MAX_WORDS: usize = 4;

/// Maximum size of one Universal MIDI Packet in bytes.
pub const UMP_MAX_PACKET_SIZE: usize = UMP_WORD_SIZE * UMP_MAX_WORDS;

/// One Universal MIDI Packet.
#[derive(Debug, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UmpPacket {
    words: [u32; UMP_MAX_WORDS],
}

impl UmpPacket {
    /// Creates a packet from its words.
    pub fn new(words: &[u32]) -> Result<Self, UmpPacketError> {
        let Some(first) = words.first() else {
            return Err(UmpPacketError::InvalidPacketLength);
        };
        let word_count = word_count((*first >> 28) as u8);
        if words.len() != word_count {
            return Err(UmpPacketError::InvalidPacketLength);
        }

        let mut packet_words = [0; UMP_MAX_WORDS];
        packet_words[..word_count].copy_from_slice(words);
        Ok(Self { words: packet_words })
    }

    /// Returns the message type.
    pub const fn message_type(&self) -> u8 {
        (self.words[0] >> 28) as u8
    }

    /// Returns the group field.
    pub const fn group(&self) -> u8 {
        ((self.words[0] >> 24) & 0x0f) as u8
    }

    /// Returns the number of valid words.
    pub const fn word_count(&self) -> usize {
        word_count(self.message_type())
    }

    /// Returns the packet words.
    pub fn words(&self) -> &[u32] {
        &self.words[..self.word_count()]
    }

    const fn byte_len(&self) -> usize {
        self.word_count() * UMP_WORD_SIZE
    }
}

/// A complete 32-bit Universal MIDI Packet.
#[derive(Debug, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ump32(u32);

impl Ump32 {
    /// Creates a 32-bit packet, rejecting message types that require more words.
    pub fn new(word: u32) -> Result<Self, UmpPacketError> {
        if word_count((word >> 28) as u8) != 1 {
            return Err(UmpPacketError::InvalidPacketLength);
        }

        Ok(Self(word))
    }

    /// Returns the message type.
    pub const fn message_type(&self) -> u8 {
        (self.0 >> 28) as u8
    }

    /// Returns the group field.
    pub const fn group(&self) -> u8 {
        ((self.0 >> 24) & 0x0f) as u8
    }

    /// Returns the packet word.
    pub const fn word(&self) -> u32 {
        self.0
    }

    /// Returns the packet word, consuming the packet.
    pub const fn into_word(self) -> u32 {
        self.0
    }
}

impl TryFrom<u32> for Ump32 {
    type Error = UmpPacketError;

    fn try_from(word: u32) -> Result<Self, Self::Error> {
        Self::new(word)
    }
}

impl From<Ump32> for UmpPacket {
    fn from(packet: Ump32) -> Self {
        Self {
            words: [packet.into_word(), 0, 0, 0],
        }
    }
}

impl TryFrom<&[u32]> for UmpPacket {
    type Error = UmpPacketError;

    fn try_from(words: &[u32]) -> Result<Self, Self::Error> {
        Self::new(words)
    }
}

impl TryFrom<&[u8]> for UmpPacket {
    type Error = UmpPacketError;

    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        let packet_len = packet_len(bytes)?;
        if bytes.len() != packet_len {
            return Err(UmpPacketError::InvalidPacketLength);
        }
        Ok(Self {
            words: decode_words(bytes),
        })
    }
}

/// Reader for iterating over Universal MIDI Packets in a USB transfer.
#[derive(Clone)]
pub struct UmpPacketReader<'a> {
    remaining: &'a [u8],
}

impl<'a> UmpPacketReader<'a> {
    /// Creates a reader over a non-empty transfer containing complete packets.
    pub fn new(data: &'a [u8]) -> Result<Self, UmpPacketError> {
        if data.is_empty() {
            return Err(UmpPacketError::InvalidPacketLength);
        }

        let mut remaining = data;
        while !remaining.is_empty() {
            let len = packet_len(remaining)?;
            remaining = remaining.get(len..).ok_or(UmpPacketError::InvalidPacketLength)?;
        }

        Ok(Self { remaining: data })
    }
}

impl Iterator for UmpPacketReader<'_> {
    type Item = UmpPacket;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining.is_empty() {
            return None;
        }

        let len = packet_len(self.remaining).ok()?;
        let (packet, remaining) = self.remaining.split_at(len);
        self.remaining = remaining;
        Some(UmpPacket {
            words: decode_words(packet),
        })
    }
}

/// Writer for accumulating Universal MIDI Packets in a USB transfer buffer.
pub struct UmpPacketWriter<'a> {
    buf: &'a mut [u8],
    position: usize,
}

impl<'a> UmpPacketWriter<'a> {
    /// Creates a writer that appends packets to `buf`.
    pub const fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, position: 0 }
    }

    /// Appends one packet to the transfer buffer.
    pub fn write(&mut self, packet: UmpPacket) -> Result<(), UmpPacketError> {
        let end = self
            .position
            .checked_add(packet.byte_len())
            .ok_or(UmpPacketError::BufferOverflow)?;
        let destination = self
            .buf
            .get_mut(self.position..end)
            .ok_or(UmpPacketError::BufferOverflow)?;

        for (destination, word) in destination.chunks_exact_mut(UMP_WORD_SIZE).zip(packet.words()) {
            destination.copy_from_slice(&word.to_le_bytes());
        }
        self.position = end;
        Ok(())
    }

    /// Returns the number of bytes written.
    pub const fn position(&self) -> usize {
        self.position
    }

    /// Returns the populated part of the transfer buffer.
    pub fn into_buf(self) -> &'a mut [u8] {
        &mut self.buf[..self.position]
    }
}

/// Universal MIDI Packet transport errors.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UmpPacketError {
    /// A packet or transfer ends before a complete UMP is available.
    InvalidPacketLength,

    /// The transfer buffer does not have room for another packet.
    BufferOverflow,
}

const fn word_count(message_type: u8) -> usize {
    const WORD_COUNTS: [usize; 16] = [1, 1, 1, 2, 2, 4, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4];
    WORD_COUNTS[(message_type & 0x0f) as usize]
}

fn packet_len(bytes: &[u8]) -> Result<usize, UmpPacketError> {
    let word = bytes.get(..UMP_WORD_SIZE).ok_or(UmpPacketError::InvalidPacketLength)?;
    let first = u32::from_le_bytes([word[0], word[1], word[2], word[3]]);
    Ok(word_count((first >> 28) as u8) * UMP_WORD_SIZE)
}

fn decode_words(bytes: &[u8]) -> [u32; UMP_MAX_WORDS] {
    let mut words = [0; UMP_MAX_WORDS];
    for (word, bytes) in words.iter_mut().zip(bytes.chunks_exact(UMP_WORD_SIZE)) {
        *word = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    }
    words
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ump32_accepts_only_complete_one_word_packets() {
        let packet = Ump32::new(0x2a90_3c7f).unwrap();
        assert_eq!(packet.message_type(), 0x2);
        assert_eq!(packet.group(), 0xa);
        assert_eq!(packet.word(), 0x2a90_3c7f);

        assert_eq!(Ump32::new(0x4a90_3c7f), Err(UmpPacketError::InvalidPacketLength));
    }

    #[test]
    fn ump32_converts_to_packet() {
        let packet = UmpPacket::from(Ump32::new(0x2090_3c7f).unwrap());
        assert_eq!(packet.words(), &[0x2090_3c7f]);
    }

    #[test]
    fn packet_validates_word_count() {
        assert_eq!(UmpPacket::new(&[0x2090_3c7f]).unwrap().word_count(), 1);
        assert_eq!(UmpPacket::new(&[0x4090_3c7f, 0x1234_5678]).unwrap().word_count(), 2);
        assert_eq!(
            UmpPacket::new(&[0xb090_3c7f, 0x1234_5678, 0x9abc_def0])
                .unwrap()
                .word_count(),
            3
        );
        assert_eq!(
            UmpPacket::new(&[0x5090_3c7f, 0x1234_5678, 0x9abc_def0, 0x1111_2222])
                .unwrap()
                .word_count(),
            4
        );

        assert_eq!(UmpPacket::new(&[0x4090_3c7f]), Err(UmpPacketError::InvalidPacketLength));
    }

    #[test]
    fn packet_from_bytes_requires_exactly_one_packet() {
        let packet = UmpPacket::try_from([0x7f_u8, 0x3c, 0x90, 0x20].as_slice()).unwrap();
        assert_eq!(packet.words(), &[0x2090_3c7f]);

        assert_eq!(
            UmpPacket::try_from([0x7f_u8, 0x3c, 0x90, 0x20, 0, 0, 0, 0].as_slice()),
            Err(UmpPacketError::InvalidPacketLength)
        );
    }

    #[test]
    fn reader_parses_mixed_packet_sizes() {
        let data = [
            0x7f, 0x3c, 0x90, 0x20, // 32-bit
            0x78, 0x56, 0x34, 0x40, 0xf0, 0xde, 0xbc, 0x9a, // 64-bit
            0x22, 0x22, 0x11, 0x50, 0x44, 0x44, 0x33, 0x33, // 128-bit
            0x66, 0x66, 0x55, 0x55, 0x88, 0x88, 0x77, 0x77,
        ];
        let mut reader = UmpPacketReader::new(&data).unwrap();

        assert_eq!(reader.next().unwrap().words(), &[0x2090_3c7f]);
        assert_eq!(reader.next().unwrap().words(), &[0x4034_5678, 0x9abc_def0]);
        assert_eq!(
            reader.next().unwrap().words(),
            &[0x5011_2222, 0x3333_4444, 0x5555_6666, 0x7777_8888]
        );
        assert_eq!(reader.next(), None);
    }

    #[test]
    fn reader_rejects_empty_or_incomplete_transfers() {
        assert!(matches!(
            UmpPacketReader::new(&[]),
            Err(UmpPacketError::InvalidPacketLength)
        ));
        assert!(matches!(
            UmpPacketReader::new(&[0x78, 0x56, 0x34]),
            Err(UmpPacketError::InvalidPacketLength)
        ));
        assert!(matches!(
            UmpPacketReader::new(&[0x78, 0x56, 0x34, 0x40]),
            Err(UmpPacketError::InvalidPacketLength)
        ));
    }

    #[test]
    fn writer_uses_usb_byte_order_and_preserves_position_on_overflow() {
        let mut buffer = [0; 9];
        let mut writer = UmpPacketWriter::new(&mut buffer);

        writer.write(UmpPacket::new(&[0x2090_3c7f]).unwrap()).unwrap();
        assert_eq!(writer.position(), 4);
        assert_eq!(
            writer.write(UmpPacket::new(&[0x4034_5678, 0x9abc_def0]).unwrap()),
            Err(UmpPacketError::BufferOverflow)
        );
        assert_eq!(writer.position(), 4);
        assert_eq!(writer.into_buf(), &[0x7f, 0x3c, 0x90, 0x20]);
    }
}
