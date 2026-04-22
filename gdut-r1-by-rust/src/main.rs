#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::usart::{Config as UartConfig, Uart};




#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! 
{

    let _p = embassy_stm32::init(Default::default());
    
    // let mut config = UartConfig::default();
    // config.baudrate = 115200;
    // config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    // config.parity = embassy_stm32::usart::Parity::ParityNone;


    // let mut uart1 = Uart::new_blocking(
    //     _p.USART1, _p.PB7, _p.PB6, config
    // ).unwrap();

    loop 
    {
        // let _ = uart1.blocking_write(b"Hello, world!\r\n");
        
        defmt::info!("Blink");
        Timer::after(Duration::from_millis(100)).await;
    }
}
