#![no_std]
#![no_main]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_rtt_target as _;
use rtt_target::{rtt_init, rprintln};

use rtt_debug_tool_mcu::Watch;
use rtt_debug_tool_mcu::watch_task::debug_watch_task;
use rtt_debug_tool_mcu::watch_table::register_watch_fields;
use rtt_debug_tool_mcu::{watch_scalar, watch_config};

// ═══════════════════════════════════════════════════════════
// StaticCell 占位 (运行前用 RefCell::new 初始化)
// ═══════════════════════════════════════════════════════════

static COUNTER: StaticCell<RefCell<u32>>  = StaticCell::new();
static VOLTAGE: StaticCell<RefCell<f32>>  = StaticCell::new();
static ACTIVE:  StaticCell<RefCell<bool>> = StaticCell::new();

// —— #[derive(Watch)] 自动注册全部字段 ——
#[derive(Watch)]
struct MotorState {
    rpm:         f32,
    current:     f32,
    temperature: u8,
}
static MOTOR: StaticCell<RefCell<MotorState>> = StaticCell::new();

#[derive(Watch)]
struct Arm {
    joint_angle: f32,
    #[watch(readonly)]
    joint_speed: f32,
}
static ARM: StaticCell<RefCell<Arm>> = StaticCell::new();

#[derive(Watch)]
struct Chassis {
    #[watch(readonly)]
    x: f32,
    #[watch(readonly)]
    y: f32,
}
static CHASSIS: StaticCell<RefCell<Chassis>> = StaticCell::new();

// ═══════════════════════════════════════════════════════════
// 主入口
// ═══════════════════════════════════════════════════════════

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // ═══ 1. RTT 多通道初始化 ═══
    let channels = rtt_init! {
        up: {
            0: { size: 1024, name: "Terminal" }
            1: { size: 1024, name: "Watch" }
        }
        down: {
            0: { size: 128, name: "Command" }
        }
    };
    rtt_target::set_print_channel(channels.up.0);

    let config = embassy_stm32::Config::default();
    embassy_stm32::init(config);

    // ═══ 2. 创建 RefCell 实例 (StaticCell → &'static RefCell) ═══
    let counter: &'static RefCell<u32> = COUNTER.init(RefCell::new(0));
    let voltage: &'static RefCell<f32> = VOLTAGE.init(RefCell::new(3.30));
    let active:  &'static RefCell<bool> = ACTIVE.init(RefCell::new(false));

    let motor: &'static RefCell<MotorState> = MOTOR.init(RefCell::new(MotorState {
        rpm:         0.0,
        current:     0.50,
        temperature: 25,
    }));

    let arm: &'static RefCell<Arm> = ARM.init(RefCell::new(Arm {
        joint_angle: 90.0,
        joint_speed: 0.0,
    }));
    
    let chassis: &'static RefCell<Chassis> = CHASSIS.init(RefCell::new(Chassis {
        x: 0.0,
        y: 0.0,
    }));

    // ═══ 3. 注册观测变量 ═══
    // 标量: watch_scalar!
    watch_scalar!("counter", counter, ReadWrite);
    watch_scalar!("voltage", voltage, ReadWrite);
    watch_scalar!("active",  active,  ReadWrite);

    // 结构体: #[derive(Watch)] + register_watch_fields (一行注册, 权限由注解决定)
    register_watch_fields("motor", motor);
    register_watch_fields("robot.arm", arm);
    register_watch_fields("robot.chassis", chassis);

    // ═══ 4. 启动 watch 后台任务 ═══
    let watch_cfg = watch_config!();
    rprintln!("[Demo] RTT Watch 调试工具演示启动");
    rprintln!("  MCU 遥测: {}Hz  宿主机: {}Hz",
        watch_cfg.mcu_freq_hz, watch_cfg.host_freq_hz);

    spawner.must_spawn(debug_watch_task(channels.up.1, channels.down.0, watch_cfg));

    // ═══ 5. 模拟运行: 周期性修改观测值 ═══
    let mut tick: u32 = 0;

    loop {
        tick += 1;

        // 计数器自增
        *counter.borrow_mut() = tick;

        // 布尔切换 (每 2 秒)
        if tick % 40 == 0 {
            let mut a = active.borrow_mut();
            *a = !*a;
        }

        // 电机: RPM 三角波 500..=1500
        let wave = if (tick / 50) % 2 == 0 {
            500.0 + (tick % 50) as f32 * 20.0
        } else {
            1500.0 - (tick % 50) as f32 * 20.0
        };
        motor.borrow_mut().rpm = wave;
        motor.borrow_mut().current = 0.3 + (tick % 100) as f32 / 100.0;
        motor.borrow_mut().temperature = 25 + (tick % 30) as u8;

        // 机械臂: 角度摆动 45°..=135°
        if (tick / 25) % 2 == 0 {
            arm.borrow_mut().joint_angle = 90.0 + (tick % 25) as f32 * 1.8;
            arm.borrow_mut().joint_speed = 30.0;
        } else {
            arm.borrow_mut().joint_angle = 135.0 - (tick % 25) as f32 * 1.8;
            arm.borrow_mut().joint_speed = -30.0;
        }

        // 底盘: 缓慢位移
        chassis.borrow_mut().x += 0.10;
        chassis.borrow_mut().y += 0.05;

        // 电压: 微小波动
        *voltage.borrow_mut() = 3.30 + (tick % 20) as f32 / 100.0;

        Timer::after(Duration::from_millis(50)).await;
    }
}
