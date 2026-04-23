#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use defmt::*; //日志输出
use embassy_stm32::{pac::sdmmc::regs::Id, peripherals::*}; //导入所有外设


// RTT日志输出 + 崩溃自动报错
use {defmt_rtt as _, panic_probe as _};


#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! 
{
    let mut config = embassy_stm32::Config::default();
    embassy_stm32::init(config);//初始化芯片
    
    loop 
    {
        info!("Hello, world!");
        Timer::after(Duration::from_secs(1)).await;
    }

}
