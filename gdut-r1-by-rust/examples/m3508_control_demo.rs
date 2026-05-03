#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_time::Timer;
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
use gdut_r1_by_rust::XieFField_Lib::motor::motor_DJI::{
    DJI_Motor, DJI_Group, SEND_ID_LOW,
};
use gdut_r1_by_rust::XieFField_Lib::motor::motor_base::{MotorCellWrapper, Motor_Base};
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
        mul: rcc::PllMul::MUL96,
        divq: Some(rcc::PllDiv::DIV4),
        divp: Some(rcc::PllDiv::DIV1),
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
    // 2. FDCAN1 硬件
    // ════════════════════════════════════════════════
    let mut can1 = can::CanConfigurator::new(
        peripherals.FDCAN1,
        peripherals.PD0,
        peripherals.PD1,
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
    // 3. 命令 Channel
    // ════════════════════════════════════════════════
    static CMD_CH: StaticCell<Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN>> =
        StaticCell::new();
    let cmd_ch: &'static Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN> =
        CMD_CH.init(Channel::new());

    // ════════════════════════════════════════════════
    // 4. fdCANbus
    // ════════════════════════════════════════════════
    static CAN1BUS: StaticCell<fdCANbus> = StaticCell::new();
    let can1_bus: &'static mut fdCANbus = CAN1BUS.init(fdCANbus::new(cmd_ch));

    // ════════════════════════════════════════════════
    // 5. 创建 3 个 M3508 电机 (RefCell<DJI_Motor> 静态分配)
    //
    // ★ 关键: 电机本体用 StaticCell<RefCell<DJI_Motor>> 分配,
    //   然后通过 MotorCellWrapper 注册到 bus, 同时把 &RefCell
    //   传给 DJI_Group 用于打包。
    // ════════════════════════════════════════════════

    // 5a. 电机1: 速度环+位置环
    static M1: StaticCell<RefCell<DJI_Motor>> = StaticCell::new();
    let m1_cell: &'static RefCell<DJI_Motor> = M1.init(RefCell::new(
        DJI_Motor::new_m3508(1, true, true)
    ));
    m1_cell.borrow_mut().init_speed_pid(PID_Param_Config::new(
        250.0, 12.0, 0.0, 15000.0, 8000.0, true, 0.1,
    ));
    m1_cell.borrow_mut().init_pos_pid(
        PID_Param_Config::new(5.0, 0.0, 0.05, 400.0, 0.0, true, 0.0),
        false, 0.0,
    );

    // 5b. 电机2: 仅速度环
    static M2: StaticCell<RefCell<DJI_Motor>> = StaticCell::new();
    let m2_cell: &'static RefCell<DJI_Motor> = M2.init(RefCell::new(
        DJI_Motor::new_m3508(2, true, false)
    ));
    m2_cell.borrow_mut().init_speed_pid(PID_Param_Config::new(
        250.0, 12.0, 0.0, 15000.0, 8000.0, true, 0.1,
    ));

    // 5c. 电机3: 纯电流
    static M3: StaticCell<RefCell<DJI_Motor>> = StaticCell::new();
    let m3_cell: &'static RefCell<DJI_Motor> = M3.init(RefCell::new(
        DJI_Motor::new_m3508(3, false, false)
    ));

    // ════════════════════════════════════════════════
    // 6. 注册单个电机到总线 (通过 MotorCellWrapper)
    //    → update / update_feedback / match_frame / CMD 都走这里
    // ════════════════════════════════════════════════
    static WRAP1: StaticCell<MotorCellWrapper<DJI_Motor>> = StaticCell::new();
    static WRAP2: StaticCell<MotorCellWrapper<DJI_Motor>> = StaticCell::new();
    static WRAP3: StaticCell<MotorCellWrapper<DJI_Motor>> = StaticCell::new();

    let w1 = WRAP1.init(MotorCellWrapper::new(m1_cell));
    let w2 = WRAP2.init(MotorCellWrapper::new(m2_cell));
    let w3 = WRAP3.init(MotorCellWrapper::new(m3_cell));

    can1_bus.register_motor(w1);
    can1_bus.register_motor(w2);
    can1_bus.register_motor(w3);

    // ════════════════════════════════════════════════
    // 7. 创建 DJI_Group (只负责 pack)
    //    → 传入 &RefCell<DJI_Motor>, Group 在 pack 时 borrow() 读取
    // ════════════════════════════════════════════════
    static DJI_GROUP1: StaticCell<DJI_Group> = StaticCell::new();
    let dji_group1 = DJI_GROUP1.init(DJI_Group::new(SEND_ID_LOW));

    dji_group1.add_motor(m1_cell);
    dji_group1.add_motor(m2_cell);
    dji_group1.add_motor(m3_cell);

    can1_bus.register_motor(dji_group1);

    info!("总线注册完成: 3xDJI_Motor + 1xDJI_Group");

    // ════════════════════════════════════════════════
    // 8. MotorHandle + 启动任务
    // ════════════════════════════════════════════════
    let h1 = MotorHandle::new(1, cmd_ch);
    let h2 = MotorHandle::new(2, cmd_ch);
    let h3 = MotorHandle::new(3, cmd_ch);

    unwrap!(spawner.spawn(bsp_fdCANbus::fdcan_bus_actor_task(rx, tx, can1_bus)));

    // ════════════════════════════════════════════════
    // 9. 控制演示
    // ════════════════════════════════════════════════
    Timer::after_millis(200).await;
    info!("===== M3508 控制演示 =====");

    // ── 电流模式 ──
    info!("[1] 电流模式: m1=2000mA, m2=1000mA, m3=500mA");
    h1.set_target_current(2000.0).await;
    h2.set_target_current(1000.0).await;
    h3.set_target_current(500.0).await;
    Timer::after_secs(2).await;

    // ── 速度模式 ──
    info!("[2] 速度模式: m1=100RPM, m2=50RPM, m3=停");
    h1.set_target_rpm(100.0).await;
    h2.set_target_rpm(50.0).await;
    h3.set_target_current(0.0).await;
    Timer::after_secs(3).await;

    // ── 位置模式 ──
    info!("[3] 位置模式: m1=90°");
    h1.set_target_angle(90.0).await;
    Timer::after_secs(3).await;

    // ── 停机 ──
    info!("[4] 全部停机");
    h1.set_target_current(0.0).await;
    h2.set_target_current(0.0).await;
    h3.set_target_current(0.0).await;

    // ════════════════════════════════════════════════
    // 10. 读取反馈: 直接 borrow RefCell<DJI_Motor>
    //
    // 应用层持有 m1_cell/m2_cell/m3_cell, 任何时候都可以:
    //   let motor = m1_cell.borrow();
    //   info!("rpm={}", motor.get_RPM());
    //
    // 因为是单核 embassy, bus task 和你的代码不会同时 borrow_mut,
    // 所以 RefCell 不会在运行时 panic。
    // ════════════════════════════════════════════════
    info!("===== 反馈读取演示 =====");

    let motor1 = m1_cell.borrow();
    info!(
        "电机1: rpm={}, current={}mA, angle={}, temp={}",
        motor1.get_RPM(),
        motor1.get_current(),
        motor1.get_angle(),
        motor1.get_temperature()
    );
    drop(motor1);

    let motor2 = m2_cell.borrow();
    info!(
        "电机2: rpm={}, target_rpm={}",
        motor2.get_RPM(),
        motor2.get_target_RPM()
    );
    drop(motor2);

    info!("===== 演示完成 =====");

    loop {
        Timer::after_secs(1).await;
        // 周期读取
        let m1 = m1_cell.borrow();
        info!("m1: rpm={}, angle={}", m1.get_RPM(), m1.get_angle());
    }
}

