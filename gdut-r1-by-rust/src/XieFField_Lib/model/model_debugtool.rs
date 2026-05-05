#![allow(dead_code)]

use core::fmt::{self, Write};

/// 调试打印器 — 栈上临时创建, 用完即弃
///
/// 实现了 `core::fmt::Write`, 所以支持标准 Rust 格式化宏:
/// `write!()`, `writeln!()`, `format_args!()`.
///
/// # 用法
/// ```ignore
/// // 1. 创建 (栈上, 随便起名)
/// let mut dbg = DebugPrinter::<128>::new();
///
/// // 2. 用 write! 宏写格式字符串
/// use core::fmt::Write;
/// write!(dbg, "motor1: rpm={:.1}, angle={:.2}\r\n", rpm, angle).ok();
///
/// // 3. 取出字节, 通过 UART 发送
/// let _ = uart_tx.write(dbg.as_bytes()).await;
/// ```
pub struct DebugPrinter<const N: usize> {
    buf: [u8; N],
    pos: usize,
}

impl<const N: usize> DebugPrinter<N> {
    pub fn new() -> Self
    {
        Self { buf: [0u8; N], pos: 0 }
    }

    /// 用 `format_args!()` 格式化, 返回待发送的字节
    ///
    /// ```ignore
    /// let data = dbg.fmt(format_args!("X:{:.3},Y:{:.3}\r\n", x, y));
    /// uart_tx.write(data).await;
    /// ```
    pub fn fmt(&mut self, args: fmt::Arguments<'_>) -> &[u8]
    {
        self.pos = 0;
        let _ = self.write_fmt(args);
        &self.buf[..self.pos]
    }

    /// 追加字节 (不清空, 用于拼装数据包)
    pub fn push_bytes(&mut self, data: &[u8])
    {
        let end = (self.pos + data.len()).min(N);
        let n = end - self.pos;
        self.buf[self.pos..end].copy_from_slice(&data[..n]);
        self.pos = end;
    }

    pub fn as_bytes(&self) -> &[u8]
    {
        &self.buf[..self.pos]
    }

    pub fn as_str(&self) -> &str
    {
        core::str::from_utf8(self.as_bytes()).unwrap_or("(utf8 err)")
    }

    pub fn clear(&mut self)
    {
        self.pos = 0;
    }

    pub fn len(&self) -> usize
    {
        self.pos
    }

    pub fn is_empty(&self) -> bool
    {
        self.pos == 0
    }
}

impl<const N: usize> Default for DebugPrinter<N> {
    fn default() -> Self
    {
        Self::new()
    }
}

impl<const N: usize> fmt::Write for DebugPrinter<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result
    {
        let bytes = s.as_bytes();
        let end = (self.pos + bytes.len()).min(N);
        let n = end - self.pos;
        self.buf[self.pos..end].copy_from_slice(&bytes[..n]);
        self.pos = end;
        Ok(())
    }
}
