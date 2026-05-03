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

use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use gdut_r1_by_rust::XieFField_Lib::bsp::bsp_fdCANbus::{
    self, fdCANbus, MotorHandle, BusCmd, CMD_QUEUE_LEN,
};
use gdut_r1_by_rust::XieFField_Lib::motor::motor_DJI::{DJI_Motor, DJI_Group, SEND_ID_LOW};
use gdut_r1_by_rust::XieFField_Lib::app::app_pid::PID_Param_Config;

use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // ════════════════════════════════════════════════
    // 1. 时钟配置
    // ════════════════════════════════════════════════
    let mut config = Config::default();

    config.rcc.hse = Some(rcc::Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });

    config.rcc.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV5,
        mul: rcc::PllMul::MUL96,       // 480MHz
        divq: Some(rcc::PllDiv::DIV4),
        divp: Some(rcc::PllDiv::DIV1), // SYSCLK = 480MHz
        divr: None,
    });

    config.rcc.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q;
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.d1c_pre = rcc::AHBPrescaler::DIV1;
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    config.rcc.apb3_pre = rcc::APBPrescaler::DIV2;
    config.rcc.apb4_pre = rcc::APBPrescaler::DIV2;
    config.rcc.voltage_scale = rcc::VoltageScale::Scale0;

    let peripherals = embassy_stm32::init(config);

    // ════════════════════════════════════════════════
    // 2. FDCAN1 硬件初始化
    // ════════════════════════════════════════════════
    let mut can1 = can::CanConfigurator::new(
        peripherals.FDCAN1,
        peripherals.PD0,  // RX
        peripherals.PD1,  // TX
        Irqs,
    );
    can1.set_bitrate(1_000_000);
    let can1 = can1.into_normal_mode();

    let props = can1.properties();
    props.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());
    props.set_extended_filter(ExtendedFilterSlot::_1, ExtendedFilter::accept_all_into_fifo1());

    let (tx, rx, _can_props) = can1.split();
    info!("FDCAN1 初始化完成, 1Mbps");

    // ════════════════════════════════════════════════
    // 3. 创建命令 Channel (独立 static)
    //
    // Channel 从 fdCANbus 中拆出, MotorHandle 和 task
    // 各持有一个 &'static 引用, 互不冲突。
    // ════════════════════════════════════════════════
    static CMD_CH: StaticCell<Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN>> =
        StaticCell::new();
    let cmd_ch: &'static Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN> =
        CMD_CH.init(Channel::new());

    // ════════════════════════════════════════════════
    // 4. 创建 fdCANbus (传入 &'static Channel)
    // ════════════════════════════════════════════════
    static CAN1BUS: StaticCell<fdCANbus> = StaticCell::new();
    let can1_bus: &'static mut fdCANbus = CAN1BUS.init(fdCANbus::new(cmd_ch));

    // ════════════════════════════════════════════════
    // 5. 创建 M3508 电机 + DJI_Group
    // ════════════════════════════════════════════════
    static DJI_GROUP1: StaticCell<DJI_Group> = StaticCell::new();
    let dji_group1 = DJI_GROUP1.init(DJI_Group::new(SEND_ID_LOW));

    // 电机1: 速度环+位置环
    let mut m1 = DJI_Motor::new_m3508(1, true, true);
    m1.init_speed_pid(PID_Param_Config::new(
        10.0, 0.5, 0.1, 5000.0, 2000.0, true, 0.0,
    ));
    m1.init_pos_pid(
        PID_Param_Config::new(5.0, 0.2, 0.05, 2000.0, 1000.0, true, 0.0),
        true,
        20.0,
    );

    // 电机2: 仅速度环
    let mut m2 = DJI_Motor::new_m3508(2, true, false);
    m2.init_speed_pid(PID_Param_Config::new(
        8.0, 0.3, 0.08, 4000.0, 1500.0, true, 0.0,
    ));

    // 电机3: 纯电流控制
    let m3 = DJI_Motor::new_m3508(3, false, false);

    dji_group1.add_motor(m1);
    dji_group1.add_motor(m2);
    dji_group1.add_motor(m3);

    info!("DJI_Group 创建完成: base_tx_id=0x{:03X}, 3xM3508", SEND_ID_LOW);

    // ════════════════════════════════════════════════
    // 6. 注册 Group 到总线
    // ════════════════════════════════════════════════
    can1_bus.register_motor(dji_group1);

    // ════════════════════════════════════════════════
    // 7. MotorHandle — 直接从 Channel 创建, 不经过 bus
    //
    // 这是消除 borrow 冲突的关键:
    //   MotorHandle 持有 &'static Channel (共享)
    //   actor task   持有 &'static mut fdCANbus (独占 motors 数组)
    //   两者各借各的, 互不干扰。
    // ════════════════════════════════════════════════
    let h1 = MotorHandle::new(1, cmd_ch);
    let h2 = MotorHandle::new(2, cmd_ch);
    let h3 = MotorHandle::new(3, cmd_ch);

    // ════════════════════════════════════════════════
    // 8. 启动总线 Actor 任务
    //
    // can1_bus 被 move 进 task, 此后外部不能再访问 bus.
    // 但 MotorHandle 早就在第7步创建好了, 不受影响。
    // ════════════════════════════════════════════════
    unwrap!(spawner.spawn(bsp_fdCANbus::fdcan_bus_actor_task(rx, tx, can1_bus)));

    // ════════════════════════════════════════════════
    // 9. 控制演示
    // ════════════════════════════════════════════════
    Timer::after_millis(200).await;
    info!("===== M3508 控制演示 =====");

    // ── 阶段1: 电流模式 ──
    info!("[1] 电流模式: 全部 500mA");
    h1.set_target_current(500.0).await;
    h2.set_target_current(500.0).await;
    h3.set_target_current(500.0).await;
    Timer::after_secs(2).await;

    // ── 阶段2: 速度模式 ──
    info!("[2] 速度模式: m1=1000RPM, m2=500RPM, m3=停");
    h1.set_target_rpm(1000.0).await;
    h2.set_target_rpm(500.0).await;
    h3.set_target_current(0.0).await;
    Timer::after_secs(3).await;

    // ── 阶段3: 角度模式 ──
    info!("[3] 角度模式: m1=90°");
    h1.set_target_angle(90.0).await;
    Timer::after_secs(3).await;

    // ── 阶段4: 总角度模式 ──
    info!("[4] 总角度模式: m1=720°(2圈)");
    h1.set_target_total_angle(720.0).await;
    Timer::after_secs(5).await;

    // ── 阶段5: 停机 ──
    info!("[5] 全部停机");
    h1.set_target_current(0.0).await;
    h2.set_target_current(0.0).await;
    h3.set_target_current(0.0).await;

    info!("===== 演示完成 =====");

    loop {
        Timer::after_secs(1).await;
        info!("心跳");
    }
}

// ════════════════════════════════════════════════════════════════
//  多路 CAN 总线扩展
// ════════════════════════════════════════════════════════════════
//
// 如果有 FDCAN1 + FDCAN2, 只需:
//
// 1. 绑两套中断:
//    bind_interrupts!(struct Irqs1 { ... });
//    bind_interrupts!(struct Irqs2 { ... });
//
// 2. 每条总线有独立的 Channel + fdCANbus + Group:
//
//    static CH1: StaticCell<Channel<...>> = StaticCell::new();
//    let ch1 = CH1.init(Channel::new());
//    static CH2: StaticCell<Channel<...>> = StaticCell::new();
//    let ch2 = CH2.init(Channel::new());
//
//    static BUS1: StaticCell<fdCANbus> = StaticCell::new();
//    let bus1 = BUS1.init(fdCANbus::new(ch1));
//    static BUS2: StaticCell<fdCANbus> = StaticCell::new();
//    let bus2 = BUS2.init(fdCANbus::new(ch2));
//
// 3. 分别 spawn (pool_size = 2 已在 task 属性中声明):
//
//    spawner.spawn(fdcan_bus_actor_task(rx1, tx1, bus1));
//    spawner.spawn(fdcan_bus_actor_task(rx2, tx2, bus2));
//
// 4. MotorHandle 用对应 Channel:
//
//    let h1 = MotorHandle::new(1, ch1); // bus1 上的电机
//    let h2 = MotorHandle::new(5, ch2); // bus2 上的电机
//
// 也可以共用一个 Channel 给两条总线:
//    - 共用: 所有 MotorHandle 发到同一个队列, 两条 bus 都会尝试 apply 命令
//      (只有注册了对应 motor_id 的那条 bus 会成功)
//    - 分开: 每条 bus 独立, 命令精准送达 (推荐)