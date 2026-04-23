#![no_std]
#![no_main]


use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use defmt::*; //日志输出
use embassy_stm32::{ peripherals:: *}; //导入所有外设

// 导入Config 系统配置 和 bind_interrupts 中断绑定 和 can can总线 和 rcc 时钟配置
use embassy_stm32::{Config, bind_interrupts, can, rcc}; 
use embassy_stm32::timer;

// RTT日志输出 + 崩溃自动报错
use {defmt_rtt as _, panic_probe as _};

use critical_section;
use embassy_stm32::pac;


bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

fn enable_cpu_freq_boost()
{
    unsafe {
        pac::RCC.apb4enr().modify(|w| w.set_syscfgen(true));
    
        let current_boost = pac::SYSCFG.ur18().read().cpu_freq_boost();
        if current_boost {
            info!("✅ CPU超频已开启，无需重复配置");
            return;
        }

        //修改前完整备份 FLASH 选项字节原始锁状态
        let orginal_lock_state = pac::FLASH.optcr().read().optlock();
        let original_optstart = pac::FLASH.optcr().read().optstart();

        if orginal_lock_state //解锁选项字节锁
        {
            pac::FLASH.optkeyr().write_value(0x08192A3B);
            pac::FLASH.optkeyr().write_value(0x4C5D6E7F);
        }

        // 等待FLASH空闲
        while pac::FLASH.bank(0).sr().read().bsy() {}

        let mut optsr = pac::FLASH.optsr_prg().read();
        optsr.set_rss2(true); // bit27 = 1 开启CPU超频
        pac::FLASH.optsr_prg().write_value( optsr);

        pac::FLASH.optcr().modify(| w| w.set_optstart(true)); //触发字节加载
        while pac::FLASH.optsr_cur().read().opt_busy() {} //等待加载完成

        pac::FLASH.optcr().modify(|w|
            {
                w.set_optlock(orginal_lock_state);
                w.set_optstart(original_optstart);
            }
        );
        
        while pac::FLASH.bank(0).sr().read().bsy() {}

        info!("✅ CPU超频已开启,将重启生效");

        
        // 读取UR18做状态验证（只读，硬件自动同步选项字节结果）
        let boost_state = pac::SYSCFG.ur18().read().cpu_freq_boost();
        info!("✅ CPU超频选项字节已配置完成,当前Boost状态: {}", boost_state);

        cortex_m::peripheral::SCB::sys_reset(); //软件复位使配置生效
    }
}


#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! 
{
    // enable_cpu_freq_boost(); //开启CPU超频功能
    let mut config = Config::default();

    //配置外部晶振时钟
    config.rcc.hse = Some(rcc::Hse{
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });


    config.rcc.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV2,   // 25/2 = 12.5MHz
        mul: rcc::PllMul::MUL44,        // 12.5*44 = 550MHz (VCO)
        divp: Some(rcc::PllDiv::DIV1),   // 550MHz → SYSCLK
        divq: Some(rcc::PllDiv::DIV2),   // 无用，随便配
        divr: None,
    });



    config.rcc.pll2 = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV5,
        mul: rcc::PllMul::MUL80,
        divp: Some(rcc::PllDiv::DIV2),
        divq: Some(rcc::PllDiv::DIV4),   // PLL2_Q = 100MHz → FDCAN时钟
        divr: None,
    });


    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.d1c_pre = rcc::AHBPrescaler::DIV1;        // D1域预分频
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;        // AHB总线 = 550MHz
    
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;       // APB1
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;       // APB2
    config.rcc.apb3_pre = rcc::APBPrescaler::DIV2;       // APB3
    config.rcc.apb4_pre = rcc::APBPrescaler::DIV2;       // APB4
    config.rcc.voltage_scale = rcc::VoltageScale::Scale0;

    //选择can的时钟源为pll1q

    config.rcc.mux.fdcansel = rcc::mux::Fdcansel::PLL2_Q;

    let peripherals = embassy_stm32::init(config);//初始化芯片

    let mut can1 = can::CanConfigurator::new(
        peripherals.FDCAN1,
        peripherals.PD0,
        peripherals.PD1,
        Irqs, //绑定前面初始化的fifo中断
        
    );
    

    can1.set_bitrate(1_000_000);

    let can1 = can1.into_normal_mode();

    info!("fdCAN1 初始化完成");

    let (tx, rx, _props) = can1.split();
    
    unwrap!(_spawner.spawn(can_tx_task(tx)));
    unwrap!(_spawner.spawn(can_rx_task(rx)));

    loop 
    {
        Timer::after(Duration::from_secs(1)).await;
    }

}

#[embassy_executor::task]
async fn can_tx_task(mut tx: can::CanTx<'static>) -> ! 
{
    let inv_real_c_2_virtual_c = 16384.0 / 20000.0;
    let send_current = (1000.0 * inv_real_c_2_virtual_c) as i16;

    let tx_data = [
        (send_current >> 8) as u8,
        (send_current & 0xFF) as u8,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];

    let tx_frame = can::Frame::new_standard(0x200, &tx_data).unwrap(); //创建发送帧

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
                    if id.as_raw() == 0x201
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