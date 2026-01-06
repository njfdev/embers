# GPS

One of the most common GPS module communication protocols is called u-blox. Embers provides a basic implementation of u-blox (tested on a HGLRC M100-5883 GPS).

## UBlox

Here is an example of how to use the UBlox driver with the Embassy framework on a Raspberry Pi Pico.

Key things to note:

- .update() must be called frequently to keep the GPS data updated and processing buffer cleared out.
- .get_new() and .get() should be called after updating to get the latest GPS information.
- To create a UBlox driver, you must create a UART interface in your environment of choice, and pass the structs implementing `Read` and `Write` from `embedded-async-io`.

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::UART1,
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx},
};
use embassy_time::Timer;
use embers::gps::ublox::{UBlox, SUGGESTED_UBLOX_BUFFER_SIZE};
use static_cell::StaticCell;

bind_interrupts!(struct UartIrq {
  UART1_IRQ => BufferedInterruptHandler<UART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // initialize UART connection
    static TX_BUF: StaticCell<[u8; SUGGESTED_UBLOX_BUFFER_SIZE]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; SUGGESTED_UBLOX_BUFFER_SIZE])[..];
    static RX_BUF: StaticCell<[u8; SUGGESTED_UBLOX_BUFFER_SIZE]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; SUGGESTED_UBLOX_BUFFER_SIZE])[..];

    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 115200; // Baud rate for HGLRC M100-5883
    let uart = BufferedUart::new(p.UART1, p.PIN_8, p.PIN_9, UartIrq, tx_buf, rx_buf, uart_config);
    let (tx, rx) = uart.split();

    // Finally initialize UBlox (everything else was Embassy specific)
    let gps = UBlox::new(rx, tx);

    loop {
      Timer::after_millis(50).await;
      let _ = gps.update().await; // may return an error (most UBlox errors are fine to ignore)

      let gps_payload = gps.get();

      // now do anything with the gps payload
    }
}
```
