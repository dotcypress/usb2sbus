#![no_std]
#![no_main]

extern crate panic_probe;
extern crate rp2040_hal as hal;
extern crate rtic;

mod bridge;

use bridge::Bridge;
use cortex_m::singleton;
use defmt_rtt as _;
use hal::fugit::RateExtU32;
use hal::gpio;
use hal::usb::UsbBus;
use hal::{clocks, gpio::Pins, pac, uart, Clock, Watchdog};
use rtic_monotonics::rp2040::prelude::*;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        bridge: Bridge,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            12_000_000,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::Sio::new(ctx.device.SIO);
        let mut pins = Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let usb_regs = ctx.device.USBCTRL_REGS;
        let usb_dpram = ctx.device.USBCTRL_DPRAM;
        let usb_bus = UsbBus::new(usb_regs, usb_dpram, clocks.usb_clock, true, &mut resets);
        let usb_bus: &'static UsbBusAllocator<UsbBus> =
            singleton!(: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(usb_bus)).unwrap();

        let info = StringDescriptors::default()
            .manufacturer("https://vitaly.codes")
            .product("USB <-> SBUS");
        let serial = SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27da))
            .strings(&[info])
            .unwrap()
            .device_class(2)
            .build();
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        }

        let uart_cfg = uart::UartConfig::new(
            100_000.Hz(),
            uart::DataBits::Eight,
            Some(uart::Parity::Even),
            uart::StopBits::Two,
        );
        pins.gpio0.set_output_override(gpio::OutputOverride::Invert);
        pins.gpio1.set_input_override(gpio::InputOverride::Invert);
        let mut uart = uart::UartPeripheral::new(
            ctx.device.UART0,
            (pins.gpio0.into_function(), pins.gpio1.into_function()),
            &mut resets,
        )
        .enable(uart_cfg, clocks.peripheral_clock.freq())
        .unwrap();
        uart.enable_rx_interrupt();

        let bridge = Bridge::new(serial, usb_dev, uart);
        (Shared { bridge }, Local {})
    }

    #[task(binds = USBCTRL_IRQ, shared = [bridge])]
    fn usb_irq(mut ctx: usb_irq::Context) {
        ctx.shared.bridge.lock(|bridge| bridge.handle_usb_irq());
    }

    #[task(binds = UART0_IRQ, shared = [bridge])]
    fn uart_irq(mut ctx: uart_irq::Context) {
        ctx.shared.bridge.lock(|bridge| bridge.handle_uart_irq());
    }
}
