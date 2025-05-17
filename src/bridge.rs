use hal::{
    gpio::{self, bank0, Pin},
    pac::{UART0, UART1},
    uart::{self, UartPeripheral},
    usb::UsbBus,
};
use usb_device::device::UsbDevice;
use usbd_serial::{
    embedded_io::{Read, Write},
    SerialPort,
};

pub type SBUSUart = UartPeripheral<
    uart::Enabled,
    UART0,
    (
        Pin<bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
        Pin<bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

pub type MavLinkUart = UartPeripheral<
    uart::Enabled,
    UART1,
    (
        Pin<bank0::Gpio4, gpio::FunctionUart, gpio::PullDown>,
        Pin<bank0::Gpio5, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

pub struct Bridge {
    usb_dev: UsbDevice<'static, UsbBus>,
    sbus_serial: SerialPort<'static, UsbBus>,
    mavlink_serial: SerialPort<'static, UsbBus>,
    sbus_uart: SBUSUart,
    mavlink_uart: MavLinkUart,
}

impl Bridge {
    pub fn new(
        usb_dev: UsbDevice<'static, UsbBus>,
        sbus_serial: SerialPort<'static, UsbBus>,
        mavlink_serial: SerialPort<'static, UsbBus>,
        sbus_uart: SBUSUart,
        mavlink_uart: MavLinkUart,
    ) -> Self {
        Self {
            usb_dev,
            sbus_serial,
            mavlink_serial,
            sbus_uart,
            mavlink_uart,
        }
    }

    pub fn handle_usb_irq(&mut self) {
        let mut scratch = [0; 128];
        if self
            .usb_dev
            .poll(&mut [&mut self.sbus_serial, &mut self.mavlink_serial])
        {
            if let Ok(n) = self.sbus_serial.read(&mut scratch) {
                self.sbus_uart.write_full_blocking(&scratch[0..n])
            }
            if let Ok(n) = self.mavlink_serial.read(&mut scratch) {
                self.mavlink_uart.write_full_blocking(&scratch[0..n])
            }
        }
    }

    pub fn handle_sbus_uart_irq(&mut self) {
        let mut scratch = [0; 128];
        if let Ok(n) = self.sbus_uart.read(&mut scratch) {
            self.sbus_serial.write_all(&scratch[0..n]).ok();
        }
    }

    pub fn handle_mavlink_uart_irq(&mut self) {
        let mut scratch = [0; 128];
        if let Ok(n) = self.mavlink_uart.read(&mut scratch) {
            self.mavlink_serial.write_all(&scratch[0..n]).ok();
        }
    }
}
