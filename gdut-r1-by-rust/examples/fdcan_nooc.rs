#![no_std]
#![no_main]

use cortex_m::peripheral;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use defmt::*; //日志输出
use embassy_stm32::{ can::config, peripherals:: *}; //导入所有外设
// 导入Config 系统配置 和 bind_interrupts 中断绑定 和 can can总线 和 rcc 时钟配置
use embassy_stm32::{Config, bind_interrupts, can, rcc}; 
use embassy_stm32::timer;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot, ExtendedFilter, ExtendedFilterSlot};
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async  fn main(_spawner: Spawner) -> !  
{
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

    let(tx, rx, _props) = can1.split();

    unwrap!(_spawner.spawn(can_tx_task(tx)));
    unwrap!(_spawner.spawn(can_rx_task(rx)));
    

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn can_tx_task(mut tx: can::CanTx<'static>) -> ! 
{
    let inv_real_c_2_virtual_c = 16384.0 / 20000.0;
    let send_current = (2000.0 * inv_real_c_2_virtual_c) as i16;

    let tx_data = [
        (send_current >> 8) as u8,
        (send_current & 0xFF) as u8,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];

    let tx_frame = can::Frame::new_standard(0x1FF, &tx_data).unwrap(); //创建发送帧

    loop {
        let _ = tx.write(&tx_frame).await;
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn can_rx_task(mut rx: can::CanRx<'static>) -> ! 
{
    loop 
    {
        match rx.read().await 
        {
            Ok(envelope) => 
            {
                let (rx_frame, _) = envelope.parts();

                if let embedded_can::Id::Standard(id) = rx_frame.id()
                {  
                    if id.as_raw() == 0x205
                    {
                        let data = rx_frame.data();
                        if data.len() >= 7 
                        {
                            let angle = u16::from_be_bytes([data[0], data[1]]);
                            let rpm = i16::from_be_bytes([data[2], data[3]]);
                            let curr_real = i16::from_be_bytes([data[4], data[5]]);
                            let temp = data[6];
                            info!("angle: {}, rpm: {}, current: {}, temp: {}", angle, rpm, curr_real, temp);
                        }
                    }
                }
            }

            Err(_err) => {}
        }
    }
}