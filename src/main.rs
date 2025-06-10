#![no_std]
#![no_main]

extern crate panic_probe;
extern crate rp2040_hal as hal;
extern crate rtic;

mod buffer;
mod config;
mod mavlink;
mod sbus;

use crate::config::*;
use crate::mavlink::MavLinkBuffer;
use crate::mavlink::MavLinkPacket;
use crate::sbus::FutabaBuffer;
use core::mem;
use cortex_m::singleton;
use defmt_rtt as _;
use embedded_io::WriteReady;
use hal::dma::{self, single_buffer};
use hal::fugit::RateExtU32;
use hal::gpio;
use hal::usb::UsbBus;
use hal::{clocks, gpio::Pins, pac, uart, Clock, Watchdog};
use rtic_monotonics::rp2040::prelude::*;
use rtic_sync::{channel::*, make_channel};
use sbus::FutabaPacket;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = pac, peripherals = true, dispatchers = [SW0_IRQ])]
mod app {
    use super::*;

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus>,
        mavlink_rx: MavLinkRx,
        mavlink_tx: Option<MavLinkSink>,
        sbus_rx: SbusRx,
        sbus_tx: Option<SbusSink>,
        sbus_sender: Sender<'static, FutabaPacket, 64>,
        sbus_receiver: Receiver<'static, FutabaPacket, 64>,
        mavlink_sender: Sender<'static, MavLinkPacket, 64>,
        mavlink_receiver: Receiver<'static, MavLinkPacket, 64>,
        sbus_vcp_sender: Sender<'static, FutabaPacket, 64>,
        sbus_vcp_receiver: Receiver<'static, FutabaPacket, 64>,
        mavlink_vcp_sender: Sender<'static, MavLinkPacket, 64>,
        mavlink_vcp_receiver: Receiver<'static, MavLinkPacket, 64>,
    }

    #[shared]
    struct Shared {
        sbus_vcp: SerialPort<'static, UsbBus>,
        mavlink_vcp: SerialPort<'static, UsbBus>,
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

        let sbus_vcp = SerialPort::new(usb_bus);
        let mavlink_vcp = SerialPort::new(usb_bus);

        let info = StringDescriptors::default()
            .manufacturer("vitaly.codes")
            .product("SBUS/MavLink");
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27da))
            .strings(&[info])
            .unwrap()
            .composite_with_iads()
            .build();

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        }

        let sbus_uart_cfg = uart::UartConfig::new(
            100_000.Hz(),
            uart::DataBits::Eight,
            Some(uart::Parity::Even),
            uart::StopBits::Two,
        );
        pins.gpio0.set_output_override(gpio::OutputOverride::Invert);
        pins.gpio1.set_input_override(gpio::InputOverride::Invert);
        let mut sbus_uart = uart::UartPeripheral::new(
            ctx.device.UART0,
            (pins.gpio0.into_function(), pins.gpio1.into_function()),
            &mut resets,
        )
        .enable(sbus_uart_cfg, clocks.peripheral_clock.freq())
        .unwrap();
        sbus_uart.enable_rx_interrupt();
        let (sbus_rx, sbus_tx) = sbus_uart.split();

        let mavlink_uart_cfg = uart::UartConfig::new(
            115_200.Hz(),
            uart::DataBits::Eight,
            None,
            uart::StopBits::One,
        );
        let mut mavlink_uart = uart::UartPeripheral::new(
            ctx.device.UART1,
            (pins.gpio4.into_function(), pins.gpio5.into_function()),
            &mut resets,
        )
        .enable(mavlink_uart_cfg, clocks.peripheral_clock.freq())
        .unwrap();
        mavlink_uart.enable_rx_interrupt();
        let (mavlink_rx, mavlink_tx) = mavlink_uart.split();

        let (sbus_sender, sbus_receiver) = make_channel!(FutabaPacket, 64);
        let (mavlink_sender, mavlink_receiver) = make_channel!(MavLinkPacket, 64);

        let (sbus_vcp_sender, sbus_vcp_receiver) = make_channel!(FutabaPacket, 64);
        let (mavlink_vcp_sender, mavlink_vcp_receiver) = make_channel!(MavLinkPacket, 64);

        let sbus_tx_buf = singleton!(: FutabaPacket = FutabaPacket::new()).unwrap();
        let mavlink_tx_buf = singleton!(: MavLinkPacket = MavLinkPacket::new()).unwrap();
        let dma = dma::DMAExt::split(ctx.device.DMA, &mut resets);
        let sbus_tx = Some(SbusSink::StandBy((dma.ch0, sbus_tx_buf, sbus_tx)));
        let mavlink_tx = Some(MavLinkSink::StandBy((dma.ch1, mavlink_tx_buf, mavlink_tx)));

        uart_arbiter::spawn().unwrap();

        (
            Shared {
                sbus_vcp,
                mavlink_vcp,
            },
            Local {
                usb_dev,
                sbus_tx,
                mavlink_tx,
                sbus_rx,
                mavlink_rx,
                sbus_sender,
                sbus_receiver,
                mavlink_sender,
                mavlink_receiver,
                sbus_vcp_sender,
                sbus_vcp_receiver,
                mavlink_vcp_sender,
                mavlink_vcp_receiver,
            },
        )
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 3,
        local = [
            usb_dev,
            sbus_sender,
            mavlink_sender,
            sbus_buffer: FutabaBuffer = FutabaBuffer::new(),
            mavlink_buffer: MavLinkBuffer = MavLinkBuffer::new(),
        ],
        shared = [sbus_vcp, mavlink_vcp]
    )]
    fn usb_irq(ctx: usb_irq::Context) {
        let usb_irq::LocalResources {
            usb_dev,
            sbus_sender,
            mavlink_sender,
            sbus_buffer,
            mavlink_buffer,
            ..
        } = ctx.local;

        let usb_irq::SharedResources {
            sbus_vcp,
            mavlink_vcp,
            ..
        } = ctx.shared;

        (sbus_vcp, mavlink_vcp).lock(|sbus_vcp, mavlink_vcp| {
            if usb_dev.poll(&mut [sbus_vcp, mavlink_vcp]) {
                let mut scratch = [0; 256];
                sbus_vcp
                    .read(&mut scratch)
                    .ok()
                    .and_then(|n| sbus_buffer.append(&scratch[0..n]))
                    .and_then(|packet| sbus_sender.try_send(packet).ok());
                mavlink_vcp
                    .read(&mut scratch)
                    .ok()
                    .and_then(|n| mavlink_buffer.append(&scratch[0..n]))
                    .and_then(|packet| mavlink_sender.try_send(packet).ok());
            }
        });
    }

    #[task(
        binds = UART0_IRQ,
        priority = 2,
        local = [
            sbus_rx,
            sbus_vcp_sender,
            sbus_buffer: FutabaBuffer = FutabaBuffer::new(),
        ]
    )]
    fn sbus_uart_irq(ctx: sbus_uart_irq::Context) {
        let sbus_uart_irq::LocalResources {
            sbus_rx,
            sbus_vcp_sender,
            sbus_buffer,
            ..
        } = ctx.local;
        let mut scratch = [0; 256];
        sbus_rx
            .read_raw(&mut scratch)
            .ok()
            .and_then(|n| sbus_buffer.append(&scratch[0..n]))
            .and_then(|packet| sbus_vcp_sender.try_send(packet).ok());
    }

    #[task(
        binds = UART1_IRQ,
        priority = 2,
        local = [
            mavlink_rx,
            mavlink_vcp_sender,
            mavlink_buffer: MavLinkBuffer = MavLinkBuffer::new(),
        ]
    )]
    fn mavlink_uart_irq(ctx: mavlink_uart_irq::Context) {
        let mavlink_uart_irq::LocalResources {
            mavlink_rx,
            mavlink_vcp_sender,
            mavlink_buffer,
            ..
        } = ctx.local;
        let mut scratch = [0; 256];
        mavlink_rx
            .read_raw(&mut scratch)
            .ok()
            .and_then(|n| mavlink_buffer.append(&scratch[0..n]))
            .and_then(|packet| mavlink_vcp_sender.try_send(packet).ok());
    }

    #[task(
        priority = 1,
        local = [
            sbus_tx,
            mavlink_tx,
            sbus_receiver,
            mavlink_receiver,
            sbus_vcp_receiver,
            mavlink_vcp_receiver,
        ],
        shared = [sbus_vcp, mavlink_vcp]
    )]
    async fn uart_arbiter(ctx: uart_arbiter::Context) {
        let uart_arbiter::LocalResources {
            sbus_tx,
            mavlink_tx,
            sbus_receiver,
            mavlink_receiver,
            sbus_vcp_receiver,
            mavlink_vcp_receiver,
            ..
        } = ctx.local;

        let uart_arbiter::SharedResources {
            mut sbus_vcp,
            mut mavlink_vcp,
            ..
        } = ctx.shared;

        let mut sbus_sink = sbus_tx.take().unwrap();
        let mut mavlink_sink = mavlink_tx.take().unwrap();

        loop {
            sbus_sink = match sbus_sink {
                SbusSink::InProgress(transfer) => {
                    if transfer.is_done() {
                        SbusSink::StandBy(transfer.wait())
                    } else {
                        SbusSink::InProgress(transfer)
                    }
                }
                SbusSink::StandBy((ch, buf, tx)) => {
                    if let Ok(packet) = sbus_receiver.try_recv() {
                        let _ = mem::replace(buf, packet);
                        SbusSink::InProgress(single_buffer::Config::new(ch, buf, tx).start())
                    } else {
                        SbusSink::StandBy((ch, buf, tx))
                    }
                }
            };

            mavlink_sink = match mavlink_sink {
                MavLinkSink::InProgress(transfer) => {
                    if transfer.is_done() {
                        MavLinkSink::StandBy(transfer.wait())
                    } else {
                        MavLinkSink::InProgress(transfer)
                    }
                }
                MavLinkSink::StandBy((ch, buf, tx)) => {
                    if let Ok(packet) = mavlink_receiver.try_recv() {
                        let _ = mem::replace(buf, packet);
                        let transfer = single_buffer::Config::new(ch, buf, tx).start();
                        MavLinkSink::InProgress(transfer)
                    } else {
                        MavLinkSink::StandBy((ch, buf, tx))
                    }
                }
            };

            if !sbus_vcp_receiver.is_empty() {
                sbus_vcp.lock(|sbus_vcp| {
                    if sbus_vcp.write_ready().unwrap_or(false) {
                        if let Ok(packet) = sbus_vcp_receiver.try_recv() {
                            sbus_vcp.write(&packet).ok();
                        }
                    }
                });
            }

            if !mavlink_vcp_receiver.is_empty() {
                mavlink_vcp.lock(|mavlink_vcp| {
                    if mavlink_vcp.write_ready().unwrap_or(false) {
                        if let Ok(packet) = mavlink_vcp_receiver.try_recv() {
                            mavlink_vcp.write(&packet).ok();
                        }
                    }
                });
            }
        }
    }
}
