use core::{
    mem,
    ops::{Deref, DerefMut},
};
use embedded_dma::ReadTarget;

#[derive(Debug)]
pub enum Error {
    BufferOverflow,
    BufferUnderflow,
}

#[derive(Debug, Clone)]
pub struct Buffer<const T: usize> {
    len: usize,
    buf: [u8; T],
}

impl<const T: usize> Buffer<T> {
    pub const fn new() -> Self {
        Self {
            len: 0,
            buf: [0; T],
        }
    }

    pub fn append(&mut self, data: &[u8]) -> Result<(), Error> {
        let new_len = self.len + data.len();
        if new_len > T {
            return Err(Error::BufferOverflow);
        }
        self.buf[self.len..new_len].copy_from_slice(data);
        self.len = new_len;
        Ok(())
    }

    pub fn pop_front(&mut self, n: usize) -> Result<Self, Error> {
        if n > self.len {
            return Err(Error::BufferUnderflow);
        }
        let mut buf = [0; T];
        buf[..n].copy_from_slice(&self.buf[..n]);
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
        Ok(Self { buf, len: n })
    }

    pub fn clear(&mut self) -> Self {
        let buf = mem::replace(&mut self.buf, [0; T]);
        let len = self.len;
        self.len = 0;
        Self { buf, len }
    }
}

impl<const T: usize> Default for Buffer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const T: usize> Deref for Buffer<T> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buf[..self.len]
    }
}

impl<const T: usize> DerefMut for Buffer<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf[..self.len]
    }
}

unsafe impl<const T: usize> ReadTarget for Buffer<T> {
    type Word = u8;

    fn as_read_buffer(&self) -> (*const Self::Word, usize) {
        let ptr = self.buf.as_ptr() as *const _;
        (ptr, self.len)
    }
}
