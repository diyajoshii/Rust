//! `embedded-hal` 0.2 → 1.0 I2C bridge.
//!
//! `tm4c123x-hal` predates `embedded-hal` 1.0 and implements the 0.2
//! blocking traits `Write`, `Read` and `WriteRead` — and only those. The
//! off-the-shelf `embedded-hal-compat` shim needs the HAL to also implement
//! `WriteIter`, `Transactional` and friends, which it does not, so its
//! blanket impl does not apply. This adapter needs exactly what the HAL has.
//!
//! 1.0's one required method is `transaction`, which takes a sequence of
//! `Operation`s to run under a single START/repeated-START. 0.2 can only
//! express `write`, `read` and `write_read`, so the sequence is coalesced
//! into those: a `Write` immediately followed by a `Read` becomes one
//! `write_read` (a repeated start on the wire); anything else runs as its
//! own transaction. That covers every pattern the driver uses.

use core::fmt::Debug;

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation, SevenBitAddress};
use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};

/// Wraps a 0.2 I2C peripheral and exposes it as a 1.0 `I2c`.
pub struct Bridge<T>(pub T);

/// The HAL's own error, carried through unchanged.
#[derive(Debug)]
pub struct BridgeError<E>(pub E);

impl<E: Debug> embedded_hal::i2c::Error for BridgeError<E> {
    fn kind(&self) -> ErrorKind {
        // The 0.2 traits give no portable way to classify errors; the inner
        // value is preserved for anyone who wants to match on the HAL type.
        ErrorKind::Other
    }
}

impl<T, E> ErrorType for Bridge<T>
where
    T: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    E: Debug,
{
    type Error = BridgeError<E>;
}

impl<T, E> I2c<SevenBitAddress> for Bridge<T>
where
    T: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    E: Debug,
{
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut i = 0;
        while i < operations.len() {
            // Split at `i` so we can hold a `&mut` to op[i] and op[i+1] at once.
            let (head, tail) = operations.split_at_mut(i + 1);
            match (&mut head[i], tail.first_mut()) {
                (Operation::Write(w), Some(Operation::Read(r))) => {
                    self.0.write_read(address, w, r).map_err(BridgeError)?;
                    i += 2;
                }
                (Operation::Write(w), _) => {
                    self.0.write(address, w).map_err(BridgeError)?;
                    i += 1;
                }
                (Operation::Read(r), _) => {
                    self.0.read(address, r).map_err(BridgeError)?;
                    i += 1;
                }
            }
        }
        Ok(())
    }
}
