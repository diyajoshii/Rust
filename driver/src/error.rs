//! Driver error type.

/// Errors returned by the driver.
///
/// Generic over the bus error `E` so that whatever the HAL reports — a NACK
/// on address versus data, an arbitration loss, a timeout — reaches the
/// caller unchanged rather than being collapsed into a unit variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The underlying I2C transaction failed. Carries the HAL's error as-is.
    Bus(E),
    /// `WHO_AM_I` did not read as an MPU-6050. Carries the value observed,
    /// which is often diagnostic on its own: `0xFF` is a floating bus, `0x00`
    /// is usually a held-low line or a device that never came out of reset.
    WrongDevice(u8),
    /// An operation that requires `init()` was called before it.
    NotInitialised,
    /// `DEVICE_RESET` did not self-clear within the poll budget.
    ResetTimeout,
    /// The requested sample rate needs an `SMPLRT_DIV` outside `0..=255` for
    /// the current DLPF setting. Carries the rate that was asked for.
    SampleRateUnreachable(u16),
    /// `FIFO_OFLOW_INT` was set. The FIFO overflowed and its contents can no
    /// longer be trusted to be packet-aligned; reset it before reading.
    FifoOverflow,
    /// `FIFO_COUNT` is not a whole multiple of the configured packet length.
    /// Carries the count. Reading would desynchronise the stream, so nothing
    /// was read.
    FifoUnaligned(u16),
    /// The caller's buffer cannot hold a single packet.
    BufferTooSmall,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}
