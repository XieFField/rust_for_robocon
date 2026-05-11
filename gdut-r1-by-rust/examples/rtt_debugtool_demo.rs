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
// 观测变量
// ═══════════════════════════════════════════════════════════

// ① f32
static BAT_VOLTAGE: StaticCell<RefCell<f32>> = StaticCell::new();

// ② i32
static LOOP_COUNT: StaticCell<RefCell<i32>> = StaticCell::new();

// ③ 普通结构体 — 一行注册全字段
#[derive(Watch)]
struct PidGains {
    kp: f32,
    ki: f32,
    #[watch(readonly)]
    kd: f32,
}
static PID: StaticCell<RefCell<PidGains>> = StaticCell::new();

// ④ 嵌套结构体 — 一行平铺所有子字段
#[derive(Watch)]
struct Motor {
    rpm:     f32,
    current: f32,
}

#[derive(Watch)]
struct Joint {
    angle: f32,
    speed: f32,
}

#[derive(Watch)]
struct Arm {
    voltage: f32,

    pitch: Motor,         // ← 自动平铺 (任何 #[derive(Watch)] 的子结构体)

    roll: Motor,

    #[watch(readonly)]
    joint: Joint,         // ← 自动平铺 + 全字段只读
}
static ARM: StaticCell<RefCell<Arm>> = StaticCell::new();

// ═══════════════════════════════════════════════════════════
// 主入口
// ═══════════════════════════════════════════════════════════

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    let bat_v: &'static RefCell<f32> = BAT_VOLTAGE.init(RefCell::new(12.60));
    let loops: &'static RefCell<i32> = LOOP_COUNT.init(RefCell::new(0));

    let pid: &'static RefCell<PidGains> = PID.init(RefCell::new(PidGains {
        kp: 2.5, ki: 0.1, kd: 0.05,
    }));

    let arm: &'static RefCell<Arm> = ARM.init(RefCell::new(Arm {
        voltage: 24.0,
        pitch:   Motor { rpm: 0.0, current: 0.0 },
        roll:    Motor { rpm: 0.0, current: 0.0 },
        joint:   Joint { angle: 90.0, speed: 0.0 },
    }));

    // ═══ 注册 — 全部自动 ═══
    watch_scalar!("battery", bat_v, ReadWrite);
    watch_scalar!("loops",   loops, ReadOnly);
    register_watch_fields("pid", pid);    // → pid.kp, pid.ki, pid.kd
    register_watch_fields("arm", arm);    // → arm.voltage, arm.pitch.rpm,
                                          //   arm.pitch.current, arm.roll.rpm,
                                          //   arm.roll.current, arm.joint.angle,
                                          //   arm.joint.speed

    // ═══ 启动 ═══
    let watch_cfg = watch_config!();
    rprintln!("[Demo] MCU {}Hz / Host {}Hz", watch_cfg.mcu_freq_hz, watch_cfg.host_freq_hz);
    spawner.must_spawn(debug_watch_task(channels.up.1, channels.down.0, watch_cfg));

    // ═══ 模拟运行 ═══
    let mut tick: i32 = 0;
    loop {
        tick += 1;
        *bat_v.borrow_mut() = 12.60 - (tick as f32) * 0.001;
        *loops.borrow_mut() = tick;

        let mut p = pid.borrow_mut();
        if tick % 100 == 0 { p.kp += 0.1; if p.kp > 5.0 { p.kp = 2.5; } }
        drop(p);

        let phase = (tick % 200) as f32;
        let mut a = arm.borrow_mut();
        a.voltage     = 24.0 + (tick % 10) as f32 * 0.1;
        a.pitch.rpm   = if phase < 100.0 { 1000.0 } else { -1000.0 };
        a.pitch.current = a.pitch.rpm.abs() * 0.002;
        a.roll.rpm    = if phase < 100.0 { -500.0 } else { 500.0 };
        a.roll.current = a.roll.rpm.abs() * 0.001;
        a.joint.angle = 90.0 + if phase < 100.0 { phase * 0.9 } else { 180.0 - phase * 0.9 };
        a.joint.speed = if phase < 100.0 { 30.0 } else { -30.0 };
        drop(a);

        Timer::after(Duration::from_millis(50)).await;
    }
}
