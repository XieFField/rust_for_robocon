#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
// use panic_halt as _;
use core::hint::black_box;
use defmt_rtt as _;
use embassy_stm32::peripherals::*;

use panic_rtt_target as _; //使用rtt输出panic信息
use rtt_target::{rtt_init_print, rprintln};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> !
{
    rtt_init_print!(); //初始化RTT日志输出
    let config = embassy_stm32::Config::default();
    embassy_stm32::init(config);//初始化芯片

    let mut id: i32 = 111;
    loop
    {
        rprintln!("Hello, world! {}", id);
        //defmt::info!("Hello, world! {}", id);
        id += 1;
        Timer::after(Duration::from_secs(1)).await;
    }
}
