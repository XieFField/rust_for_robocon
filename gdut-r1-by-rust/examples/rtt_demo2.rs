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

#[derive(Watch)]
struct test1{
    a: f32,
    b: i32,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> !{

    let channels = rtt_init! {
        up: {
            0: { size: 1024, name: "Terminal" }
            1: { size: 1024, name: "Watch" }
        }
        down: {
            0: { size: 128, name: "Command" }
        }
    };


    loop {
        
    }
}