#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_stm32::{pac::sdmmc::regs::Id, peripherals::*}; //导入所有外设

use gdut_r1_by_rust::XieFField_Lib::app::pid::{self, PID_Incremental, PID_Param_Config};
use gdut_r1_by_rust::XieFField_Lib::app::pid::PID_Position;
use {defmt_rtt as _, panic_probe as _};
use defmt::*; //日志输出

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! 
{

    let pid_pos_param = PID_Param_Config::new(1.0, 0.5, 0.1, 100.0, 10.0, true, 0.0);
    
    let mut pid_pos = PID_Position::new(
        pid_pos_param, true, 0.0, 0.01);

    let mut config = embassy_stm32::Config::default();
    embassy_stm32::init(config);//初始化芯片

    let mut output = 0.0;
    loop 
    {

        output = pid_pos.pid_calc(90.0, output);

        info!("Hello, world!");
        Timer::after(Duration::from_secs(1)).await;
    }

}