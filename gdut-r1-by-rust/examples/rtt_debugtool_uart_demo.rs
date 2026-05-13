//! RTT Debug Tool — UART 串口版演示
//!
//! 接线: PA9(TX) → USB转串口RX, PA10(RX) → USB转串口TX, GND → GND
//! 启动: spawner.must_spawn(debug_watch_task_uart(uart, config));

#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_rtt_target as _;
use rtt_target::{rtt_init, rprintln};

use rtt_debug_tool_mcu::Watch;
use rtt_debug_tool_mcu::watch_task::debug_watch_task_uart;
use rtt_debug_tool_mcu::watch_table::register_watch_fields;
use rtt_debug_tool_mcu::{watch_scalar, watch_config};


use embassy_stm32::usart::{self, RxPin, TxPin, Uart};
use embassy_stm32::mode::Async;
use embassy_stm32::{bind_interrupts, dma, peripherals};

bind_interrupts!(struct Irqs { 
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
});

// ═══════════════════════════════════════════════════════════
// 观测变量 (与 RTT demo 相同)
// ═══════════════════════════════════════════════════════════

static BAT_VOLTAGE: StaticCell<RefCell<f32>> = StaticCell::new();
static LOOP_COUNT: StaticCell<RefCell<i32>> = StaticCell::new();

#[derive(Watch)]
struct PidGains {
    kp: f32,
    ki: f32,
    #[watch(readonly)]
    kd: f32,
}
static PID: StaticCell<RefCell<PidGains>> = StaticCell::new();

#[derive(Watch)]
struct Motor { rpm: f32, current: f32 }

#[derive(Watch)]
struct Joint { angle: f32, speed: f32 }

#[derive(Watch)]
struct Arm {
    voltage: f32,
    pitch: Motor,
    roll: Motor,
    #[watch(readonly)]
    joint: Joint,
}
static ARM: StaticCell<RefCell<Arm>> = StaticCell::new();

// ═══════════════════════════════════════════════════════════
// 主入口
// ════════════════════════════════════════════════
#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // RTT 仅用于 rprintln! 日志 (ch0), 不需要 watch 通道
    let channels = rtt_init! {
        up: { 0: { size: 1024, name: "Terminal" } }
    };
    rtt_target::set_print_channel(channels.up.0);

    let config = embassy_stm32::Config::default();
    let p = embassy_stm32::init(config);

    // ── 初始化 UART (不 split, 直接用) ──
    let mut uart_config = usart::Config::default();
    uart_config.baudrate = 115200;
    uart_config.parity = usart::Parity::ParityNone;
    uart_config.stop_bits = usart::StopBits::STOP1;

    let uart = Uart::new(p.UART8,
        p.PE0, p.PE1, p.DMA1_CH7, p.DMA2_CH2, Irqs, uart_config).unwrap();
    // ── 创建 RefCell 实例 ──
    let bat_v = BAT_VOLTAGE.init(RefCell::new(12.60));
    let loops = LOOP_COUNT.init(RefCell::new(0));
    let pid = PID.init(RefCell::new(PidGains { kp: 2.5, ki: 0.1, kd: 0.05 }));
    let arm = ARM.init(RefCell::new(Arm {
        voltage: 24.0,
        pitch: Motor { rpm: 0.0, current: 0.0 },
        roll:  Motor { rpm: 0.0, current: 0.0 },
        joint: Joint { angle: 90.0, speed: 0.0 },
    }));

    // ── 注册 ──
    watch_scalar!("battery", bat_v, ReadWrite);
    watch_scalar!("loops",   loops, ReadOnly);
    register_watch_fields("pid", pid);
    register_watch_fields("arm", arm);

    // ── 启动 UART watch 任务 (直接 spawn, 和 RTT 版接口一致) ──
    spawner.must_spawn(debug_watch_task_uart(uart, watch_config!()));

    rprintln!("[UART Demo] 启动");

    // ── 模拟运行 (与 RTT demo 相同) ──
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
        a.voltage = 24.0 + (tick % 10) as f32 * 0.1;
        a.pitch.rpm = if phase < 100.0 { 1000.0 } else { -1000.0 };
        a.pitch.current = a.pitch.rpm.abs() * 0.002;
        a.roll.rpm = if phase < 100.0 { -500.0 } else { 500.0 };
        a.roll.current = a.roll.rpm.abs() * 0.001;
        a.joint.angle = 90.0 + if phase < 100.0 { phase * 0.9 } else { 180.0 - phase * 0.9 };
        a.joint.speed = if phase < 100.0 { 30.0 } else { -30.0 };
        drop(a);
        Timer::after(Duration::from_millis(50)).await;
    }
}
