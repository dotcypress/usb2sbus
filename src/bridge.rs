use hal::{
    gpio::{self, bank0, Pin},
    pac::UART0,
    uart::{self, UartPeripheral},
    usb::UsbBus,
};
use usb_device::device::{UsbDevice, UsbDeviceState};
use usbd_serial::{
    embedded_io::{Read, Write},
    SerialPort,
};

pub type Uart = UartPeripheral<
    uart::Enabled,
    UART0,
    (
        Pin<bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
        Pin<bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

pub struct Bridge {
    serial: SerialPort<'static, UsbBus>,
    usb_dev: UsbDevice<'static, UsbBus>,
    uart: Uart,
}

impl Bridge {
    pub fn new(
        serial: SerialPort<'static, UsbBus>,
        usb_dev: UsbDevice<'static, UsbBus>,
        uart: Uart,
    ) -> Self {
        Self {
            serial,
            usb_dev,
            uart,
        }
    }

    pub fn handle_usb_irq(&mut self) {
        if self.usb_dev.poll(&mut [&mut self.serial]) {
            let mut scratch = [0; 64];
            if let Ok(n) = self.serial.read(&mut scratch) {
                self.uart.write_full_blocking(&scratch[0..n])
            }
        }
    }

    pub fn handle_uart_irq(&mut self) {
        let mut scratch = [0; 64];
        if let Ok(n) = self.uart.read(&mut scratch) {
            if self.usb_dev.state() == UsbDeviceState::Configured {
                self.serial.write_all(&scratch[0..n]).ok();
            }
        }
    }
}
