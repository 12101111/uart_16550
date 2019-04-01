#![no_std]
use bitflags::bitflags;
use x86_64::instructions::port::{Port, PortReadOnly};
/// This crate provide a abstraction of the UART 16550 chip on PC.
/// Only send message is working now.
/// 
/// See also:
/// https://en.wikibooks.org/wiki/Serial_Programming/8250_UART_Programming
/// https://www.lammertbies.nl/comm/info/serial-uart.html
/// https://wiki.osdev.org/UART

/// Baud speed
///
/// speed lower than 1200 (300, 50) is rarely seen
pub enum Speed {
    B1200 = 96,
    B2400 = 48,
    B4800 = 24,
    B9600 = 12,
    B19200 = 6,
    B38400 = 3,
    B57600 = 2,
    B115200 = 1,
}

/// TODO: We barely see such a device with 5 bits or 6 bits word length or non ASCII code
/// So don't support 5 bits or 6 bits
/// https://retrocomputing.stackexchange.com/questions/2297/what-were-the-applications-of-5-6-bit-serial-port-formats
pub enum WordLength {
    Bits7 = 2,
    Bits8 = 3,
}

pub enum Parity {
    None = 0,
    Odd = 1,
    Even = 3,
    Mark = 5,
    Space = 7,
}

bitflags! {
    struct IntEnFlags: u8 {
        /// Received data available
        const DATA_AVALIABLE = 1 << 0;
        /// Transmitter holding register empty
        const TRANSMITTER_EMPTY = 1 << 1;
        /// Line status register change
        const LINE_STATUS_CHANGE = 1 << 2;
        /// Modem status register change
        const MODEM_STATUS_CHANGE = 1 << 3;
    }
}

pub enum InterruptIdent {
    /// Modem status change, Reset by MSR read
    ModemStatusChange = 0b000,
    /// Transmitter holding register empty, Reset by IIR read or THR write
    TransmitterEmpty = 0b001,
    /// Received data available, Reset by RBR read
    DataAvailable = 0b010,
    /// Line status change, Reset by LSR read
    LineStatusChange = 0b011,
    /// Character timeout (16550), Reset by RBR read
    CharacterTimeout = 0b110,
}

bitflags! {
    struct LineStatusFlags : u8{
        const DATA_AVALIABLE = 1 << 0;
        const OVERRUN_ERROR = 1 << 1;
        const PARITY_ERROR = 1 << 2;
        const FRAMING_ERROR = 1 << 3;
        /// Break signal received
        const BREAK_INTERRUPT = 1 << 4;
        /// THR is empty
        const EMPTY_TRANSMITTER = 1 << 5;
        /// THR is empty, and line is idle
        const EMPTY_RECV_DATA = 1 << 6;
        /// Errornous data in FIFO
        const ERROR_IN_FIFO = 1 << 7;
    }
}

/// UART in PC is a sets of ports in I/O address space.
/// Standard 16550 UART have 8 ports.
/// This is a wrapper around them.
pub struct UART {
    /// +0 Read:  RBR Transmitter Holding Buffer
    ///    Write: THR Receiver Buffer
    buffer: Port<u8>,
    /// +1 IER interrupt enable
    int_en: Port<u8>,
    /// +2 Read:  IIR interrupt identification
    ///    Write: FCR FIFO control
    int_ident_fifo_ctrl: Port<u8>,
    /// +3 LCR line control
    line_ctrl: Port<u8>,
    /// +4 MCR modem control
    modem_ctrl: Port<u8>,
    /// +5 LSR line status. Read Only
    line_status: PortReadOnly<u8>,
    // +6 MSR modem status. Read Only
    //modem_status: PortReadOnly<u8>,
    // +7 SCR scratch
    //scratch: Port<u8>,
}

impl UART {
    /// Get COM 1-4 base on regular PC I/O address.
    ///
    /// This function is unsafe because it write and read scratch port,
    /// which could have side effects that violate memory safety.
    ///
    /// Return None if you pass a number other than 1-4
    /// or test on scratch port failed.
    pub unsafe fn new(com: u8) -> Option<UART> {
        let addr = Self::get_base_addr(com)?;
        let mut scratch = Port::new(addr + 7);
        if !Self::test_addr(&mut scratch) {
            None
        } else {
            Some(Self::from_base_addr(addr))
        }
    }

    /// Get a 16550 UART in provided address.
    ///
    /// This function is unsafe because the address may be invalid.
    /// Reading or writing invalid I/O address may cause critical fault.
    /// You should provided a correct address or test it carefully.
    pub fn from_base_addr(addr: u16) -> UART {
        UART {
            buffer: Port::new(addr),
            int_en: Port::new(addr + 1),
            int_ident_fifo_ctrl: Port::new(addr + 2),
            line_ctrl: Port::new(addr + 3),
            modem_ctrl: Port::new(addr + 4),
            line_status: PortReadOnly::new(addr + 5),
            //modem_status: PortReadOnly::new(addr + 6),
            //scratch: Port::new(addr + 7),
        }
    }

    fn get_base_addr(com: u8) -> Option<u16> {
        match com {
            1 => Some(0x3F8),
            2 => Some(0x2F8),
            3 => Some(0x3E8),
            4 => Some(0x2E8),
            _ => None,
        }
    }

    // early version of 8250 don't pass this test
    // Beacuse it don't have Scratch Register
    // Since this crate only work with 16550+ (FIFO support)
    // We use it to detect 16550+
    unsafe fn test_addr(scratch: &mut Port<u8>) -> bool {
        const TEST_VAL: u8 = 0x2A;
        scratch.write(TEST_VAL);
        let result = scratch.read();
        result == TEST_VAL
    }

    /// initialize with default setting
    ///
    /// baud speed: 115200, word length: 8 Bits, 1 Stop bit, parity: none
    pub fn init_with_default(&mut self) {
        self.init(
            Speed::B115200,
            WordLength::Bits8,
            false,
            Parity::None,
            false,
        )
    }

    /// initialize with default line setting and a specific baud speed.
    ///
    /// other setting: word length: 8 Bits, 1 Stop bit, parity: none
    pub fn init_with_baud(&mut self, baud: Speed) {
        self.init(baud, WordLength::Bits8, false, Parity::None, false)
    }

    /// initialize this UART
    ///
    /// two_stop_bit:
    ///
    /// ture: 1.5 (for 5bits word length) / 2 stop bits
    /// false: 1 stop bit
    ///
    pub fn init(
        &mut self,
        baud: Speed,
        word_length: WordLength,
        two_stop_bit: bool,
        parity: Parity,
        enable_break_signal: bool,
    ) {
        unsafe {
            // disable all interrupt
            self.int_en.write(0x00);
            // dlab = 1;
            self.line_ctrl.write(0x80);
            // divisor latch LSB
            self.buffer.write(baud as u8);
            // divisor latch MSB
            self.int_en.write(0x00);
            // dlab = 0,set word length, bit stop, parity
            let val = (word_length as u8)
                | ((two_stop_bit as u8) << 2)
                | ((parity as u8) << 3)
                | ((enable_break_signal as u8) << 6);
            self.line_ctrl.write(val);
            // Enable FIFO with 14-byte threshold, clear receive and transmit FIFO
            self.int_ident_fifo_ctrl.write(0xC7);
            // IRQs enabled, RTS/DSR set
            self.modem_ctrl.write(0x0B);
            // TODO: should this enable?
            // enable interrupt on data available
            self.int_en.write(0x01);
        }
    }

    /// clear receive fifo
    pub fn clear_receive(&mut self) {
        unsafe {
            self.int_ident_fifo_ctrl.write(0xC3);
        }
    }

    /// clear transmit fifo
    pub fn clear_transmit(&mut self) {
        unsafe {
            self.int_ident_fifo_ctrl.write(0xC5);
        }
    }

    // TODO: interrupt support
    fn get_int_ident(&mut self) -> Option<InterruptIdent> {
        use InterruptIdent::*;
        let val = unsafe { self.int_ident_fifo_ctrl.read() };
        if val & 0x1 == 0x1 {
            None
        } else {
            match (val >> 1) & 0x7 {
                0b000 => Some(ModemStatusChange),
                0b001 => Some(TransmitterEmpty),
                0b010 => Some(DataAvailable),
                0b011 => Some(LineStatusChange),
                0b110 => Some(CharacterTimeout),
                _ => None,
            }
        }
    }

    fn get_line_status(&mut self) -> LineStatusFlags {
        unsafe { LineStatusFlags::from_bits_truncate(self.line_status.read()) }
    }

    /// Send a char
    pub fn send(&mut self, data: u8) {
        match data {
            // 0x8 is Back space and 0x7F is Delete
            // https://stackoverflow.com/questions/6792812/the-backspace-escape-character-b-unexpected-behavior
            0x8 | 0x7F => {
                self.putc(0x8);
                self.putc(b' ');
                self.putc(0x8);
            }
            _ => self.putc(data),
        }
    }

    fn putc(&mut self, data: u8) {
        unsafe {
            while !self
                .get_line_status()
                .contains(LineStatusFlags::EMPTY_TRANSMITTER)
            {}
            self.buffer.write(data);
        }
    }
}

impl core::fmt::Write for UART {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            self.send(byte);
        }
        Ok(())
    }
}
