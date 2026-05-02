#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
use defmt::*;
use embassy_stm32::peripherals::*;
use embassy_stm32::{Config, bind_interrupts, can, rcc};
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot, ExtendedFilter, ExtendedFilterSlot};

use gdut_r1_by_rust::XieFField_Lib::bsp::bsp_fdCANbus::{
    self, fdCANbus, MotorHandle,
};
use gdut_r1_by_rust::XieFField_Lib::motor::motor_DJI::{
    DJI_Motor, DJI_Group, SEND_ID_LOW
};

use static_cell::StaticCell;

// ── FDCAN 中断绑定 ──────────────────────────────────
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});


// ── 主入口 ──────────────────────────────────────────
#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = Config::default();

    //配置外部晶振时钟
    config.rcc.hse = Some(
        rcc::Hse{
            freq: embassy_stm32::time::Hertz(25_000_000), //外部晶振频率
            mode: rcc::HseMode::Oscillator, //晶振模式
        }
    );

    config.rcc.pll1 = Some(
        rcc::Pll{
            source: rcc::PllSource::HSE, //PLL时钟源选择外部晶振
            prediv: rcc::PllPreDiv::DIV5, //PLL预分频系数   
            mul: rcc::PllMul::MUL96,       //480Mhz
            divq: Some(rcc::PllDiv::DIV4), 
            divp: Some(rcc::PllDiv::DIV1), //PLL1_P = 480MHz → SYSCLK
            divr: None, //无用，随便配
        }
    );

    config.rcc.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q; //选择can的时钟源为pll1q

    config.rcc.sys = rcc::Sysclk::PLL1_P; //系统时钟源选择pll1p
    config.rcc.d1c_pre = rcc::AHBPrescaler::DIV1;        // D1域预分频
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;        // AHB总线 = 480MHz / 2 = 240MHz
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;       // APB1 = 120MHz
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;       // APB2 = 120MHz
    config.rcc.apb3_pre = rcc::APBPrescaler::DIV2;       // APB3 = 120MHz
    config.rcc.apb4_pre = rcc::APBPrescaler::DIV2;       // APB4 = 120MHz
    config.rcc.voltage_scale = rcc::VoltageScale::Scale0; 

    let peripherals = embassy_stm32::init(config);

    let mut can1 = can::CanConfigurator::new(
        peripherals.FDCAN1,
        peripherals.PD0,
        peripherals.PD1,
        Irqs, //绑定前面初始化的fifo中断
    );


    can1.set_bitrate(1_000_000);

    let can1 = can1.into_normal_mode();
    let props = can1.properties();
    props.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());
    // 扩展帧全接收（过滤器1）
    props.set_extended_filter(ExtendedFilterSlot::_1, ExtendedFilter::accept_all_into_fifo1());
    

    info!("FDCAN1 initialized with 1Mbps bitrate");

    static CAN1BUS: StaticCell<fdCANbus> = StaticCell::new();
    let can1_bus: &'static mut fdCANbus = CAN1BUS.init(fdCANbus::new());

    static DJI_GROUP1: StaticCell<DJI_Group> = StaticCell::new();
    let dji_group1 = DJI_GROUP1.init(DJI_Group::new(SEND_ID_LOW));

    //创建电机对象并注册到总线
    let m35081_1 = DJI_Motor::new_m3508(1, true, false);
    let m35081_2 = DJI_Motor::new_m3508(2, true, false);
    let m35081_3 = DJI_Motor::new_m3508(3, true, false);
    
    dji_group1.add_motor(m35081_1);
    dji_group1.add_motor(m35081_2);
    dji_group1.add_motor(m35081_3);

    can1_bus.register_motor(dji_group1);

     //将can总线的发送接口传给电机控制组

    loop {
        Timer::after_secs(1).await;
        info!("心跳: 系统运行中");
    }
}

