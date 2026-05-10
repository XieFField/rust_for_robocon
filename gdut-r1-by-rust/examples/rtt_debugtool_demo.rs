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
use rtt_debug_tool_mcu::{watch_scalar, watch_struct, watch_struct_all, watch_config};

// ═══════════════════════════════════════════════════════════
// 观测变量定义
// ═══════════════════════════════════════════════════════════

// ── 1. 普通 f32 变量 ──
static BAT_VOLTAGE: StaticCell<RefCell<f32>> = StaticCell::new();

// ── 2. 普通 i32 变量 ──
static LOOP_COUNT: StaticCell<RefCell<i32>> = StaticCell::new();

// ── 3. 非嵌套结构体 (手动选择字段 + 精确指定读写权限) ──
struct PidGains {
    kp: f32,
    ki: f32,
    kd: f32,
}
static PID: StaticCell<RefCell<PidGains>> = StaticCell::new();

// ── 4. 嵌套结构体 (自动展开全部子字段) ──
//      内部: #[derive(Watch)] 自动发现 JointState 的字段
//      外部: watch_struct_all! 将 pitch.* / roll.* 平铺为观测条目

#[derive(Watch)]
struct JointState {
    angle: f32,
    #[watch(readonly)]
    speed: f32,
}

struct RobotArm {
    voltage: f32,
    pitch:   JointState,
    roll:    JointState,
}
static ARM: StaticCell<RefCell<RobotArm>> = StaticCell::new();

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

    // ═══ 2. 创建 RefCell 实例 ═══
    let bat_v:  &'static RefCell<f32> = BAT_VOLTAGE.init(RefCell::new(12.60));
    let loops:  &'static RefCell<i32> = LOOP_COUNT.init(RefCell::new(0));

    let pid: &'static RefCell<PidGains> = PID.init(RefCell::new(PidGains {
        kp: 2.5,
        ki: 0.1,
        kd: 0.05,
    }));

    let arm: &'static RefCell<RobotArm> = ARM.init(RefCell::new(RobotArm {
        voltage: 24.0,
        pitch:   JointState { angle: 45.0,  speed: 0.0 },
        roll:    JointState { angle: -10.0, speed: 0.0 },
    }));

    // ═══ 3. 注册观测变量 ═══

    // 普通变量: watch_scalar!
    watch_scalar!("battery", bat_v, ReadWrite);
    watch_scalar!("loops",   loops, ReadOnly);

    // 非嵌套结构体: watch_struct! — 手动选择字段、精确指定权限
    watch_struct!("pid", PidGains, pid, {
        kp: f32 => ReadWrite,
        ki: f32 => ReadWrite,
        kd: f32 => ReadOnly,
    });

    // 嵌套结构体: watch_struct_all! — 平铺所有子字段, 默认 ReadWrite
    // pitch.* / roll.* 会展开为 arm.pitch.angle, arm.pitch.speed ...
    watch_struct_all!("arm", RobotArm, arm, {
        voltage:      f32,
        pitch.angle:  f32 => ReadOnly,
        pitch.speed:  f32 => ReadOnly,
        roll.angle:   f32 => ReadOnly,
        roll.speed:   f32 => ReadOnly,
    });

    // ═══ 4. 启动 watch 后台任务 ═══
    let watch_cfg = watch_config!();
    rprintln!("[Demo] RTT Watch 启动: MCU {}Hz / Host {}Hz",
        watch_cfg.mcu_freq_hz, watch_cfg.host_freq_hz);

    spawner.must_spawn(debug_watch_task(channels.up.1, channels.down.0, watch_cfg));

    // ═══ 5. 模拟运行: 周期性修改观测值 ═══
    let mut tick: i32 = 0;

    loop {
        tick += 1;

        // 电池电压: 缓慢下降
        *bat_v.borrow_mut() = 12.60 - (tick as f32) * 0.001;

        // 循环计数
        *loops.borrow_mut() = tick;

        // PID 参数: 模拟手动调参
        let mut p = pid.borrow_mut();
        if tick % 100 == 0 {
            p.kp += 0.1;
            if p.kp > 5.0 { p.kp = 2.5; }
        }
        drop(p);

        // 机械臂: 周期性摆动
        let phase = (tick % 200) as f32;
        let mut a = arm.borrow_mut();
        a.voltage = 24.0 + (tick % 10) as f32 * 0.1;
        a.pitch.angle = 45.0 + (phase * 1.8);
        a.pitch.speed = if phase < 100.0 { 30.0 } else { -30.0 };
        a.roll.angle  = -10.0 + (phase * 0.9 - 90.0);
        a.roll.speed  = if phase < 100.0 { -15.0 } else { 15.0 };
        drop(a);

        Timer::after(Duration::from_millis(50)).await;
    }
}
