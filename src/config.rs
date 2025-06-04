use crate::mavlink::MavLinkPacket;
use crate::sbus::FutabaPacket;
use rp2040_hal::gpio::*;
use rp2040_hal::*;

pub type SbusTxPin = hal::gpio::Pin<bank0::Gpio0, FunctionUart, PullDown>;
pub type SbusRxPin = hal::gpio::Pin<bank0::Gpio1, FunctionUart, PullDown>;

pub type MavLinkTxPin = hal::gpio::Pin<bank0::Gpio4, FunctionUart, PullDown>;
pub type MavLinkRxPin = hal::gpio::Pin<bank0::Gpio5, FunctionUart, PullDown>;

pub type SbusRx = uart::Reader<pac::UART0, (SbusTxPin, SbusRxPin)>;
pub type SbusTx = uart::Writer<pac::UART0, (SbusTxPin, SbusRxPin)>;

pub type MavLinkRx = uart::Reader<pac::UART1, (MavLinkTxPin, MavLinkRxPin)>;
pub type MavLinkTx = uart::Writer<pac::UART1, (MavLinkTxPin, MavLinkRxPin)>;

pub type SbusSinkConfig = (
    hal::dma::Channel<hal::dma::CH0>,
    &'static mut FutabaPacket,
    SbusTx,
);

pub type MavLinkSinkConfig = (
    hal::dma::Channel<hal::dma::CH1>,
    &'static mut MavLinkPacket,
    MavLinkTx,
);

pub type SbusSinkTransfer =
    dma::single_buffer::Transfer<dma::Channel<dma::CH0>, &'static mut FutabaPacket, SbusTx>;

pub type MavLinkTransfer = dma::single_buffer::Transfer<
    hal::dma::Channel<hal::dma::CH1>,
    &'static mut MavLinkPacket,
    MavLinkTx,
>;

pub enum SbusSink {
    StandBy(SbusSinkConfig),
    InProgress(SbusSinkTransfer),
}

pub enum MavLinkSink {
    StandBy(MavLinkSinkConfig),
    InProgress(MavLinkTransfer),
}
