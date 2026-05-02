use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::can;
use embedded_can::Id;

use crate::XieFField_Lib::motor::motor_base::Motor_Base;


#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    pub id: u32,
    pub data: [u8; 8],
    pub dlc: u8,
    pub is_extended: bool,
}

impl CanFrame {
    pub fn new(id: u32, is_extended: bool) -> Self 
    {
        Self {
            id,
            is_extended: is_extended,
            dlc: 8,
            data: [0u8; 8],
        }
    }
}

pub(crate) const MAX_MOTORS: usize = 10;
pub(crate) const DEFAULT_CONTROL_HZ: u16 = 1000;
pub(crate) const TX_FRAME_BUF_LEN: usize = 32;
pub(crate) const CMD_QUEUE_LEN: usize = 16;

#[derive(Clone, Copy)]
pub enum CommonCmd {
    SetTargetRpm { motor_id: u32, rpm: f32 },
    SetTargetAngle { motor_id: u32, angle: f32 },
    SetTargetCurrent { motor_id: u32, current: f32 },
    SetTargetTotalAngle { motor_id: u32, total_angle: f32 },
}

#[derive(Clone, Copy)]
pub enum DjiCmd {
    RelocateTotalAngle { motor_id: u32, value: f32 },
}

#[derive(Clone, Copy)]
pub enum BusCmd {
    Common(CommonCmd),
    Dji(DjiCmd),
}

#[derive(Clone, Copy)]
pub struct MotorHandle {
    motor_id: u32,
    cmd_ch: &'static Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN>,
}

impl MotorHandle {
    pub const fn new(
        motor_id: u32,
        cmd_ch: &'static Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN>,
    ) -> Self 
    {
        Self { motor_id, cmd_ch }
    }

    pub async fn set_target_rpm(&self, rpm: f32) 
    {
        self.cmd_ch
            .send(BusCmd::Common(CommonCmd::SetTargetRpm 
                {
                motor_id: self.motor_id,
                rpm,
            }))
            .await;
    }

    pub async fn set_target_angle(&self, angle: f32) 
    {
        self.cmd_ch
            .send(BusCmd::Common(CommonCmd::SetTargetAngle {
                motor_id: self.motor_id,
                angle,
            }))
            .await;
    }

    pub async fn set_target_current(&self, current: f32) 
    {
        self.cmd_ch
            .send(BusCmd::Common(CommonCmd::SetTargetCurrent {
                motor_id: self.motor_id,
                current,
            }))
            .await;
    }

    pub async fn set_target_total_angle(&self, total_angle: f32) 
    {
        self.cmd_ch
            .send(BusCmd::Common(CommonCmd::SetTargetTotalAngle {
                motor_id: self.motor_id,
                total_angle,
            }))
            .await;
    }

    pub async fn dji_relocate_total_angle(&self, value: f32) 
    {
        self.cmd_ch
            .send(BusCmd::Dji(DjiCmd::RelocateTotalAngle {
                motor_id: self.motor_id,
                value,
            }))
            .await;
    }
}

pub struct fdCANbus {
    motors: [Option<&'static mut dyn Motor_Base>; MAX_MOTORS],
    cmd_ch: Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN>,
}

impl fdCANbus {
    pub fn new() -> Self 
    {
        Self {
            motors: [None, None, None, None, None, None, None, None, None, None,],
            cmd_ch: Channel::new(),
        }
    }

    pub fn cmd_channel(&'static self) -> &'static Channel<CriticalSectionRawMutex, BusCmd, CMD_QUEUE_LEN> 
    {
        &self.cmd_ch
    }

    pub fn motor_handle(&'static self, motor_id: u32) -> MotorHandle 
    {
        MotorHandle::new(motor_id, self.cmd_channel())
    }

    pub fn register_motor(&mut self, motor: &'static mut dyn Motor_Base) -> bool 
    {
        for slot in self.motors.iter_mut() 
        {
            if slot.is_none() 
            {
                *slot = Some(motor);
                return true;
            }
        }
        false
    }
}

fn to_canframe(rx_frame: &can::Frame) -> CanFrame 
{
    let mut data = [0u8; 8];
    let payload = rx_frame.data();
    let dlc = payload.len().min(8);
    data[..dlc].copy_from_slice(&payload[..dlc]);

    match rx_frame.id() 
    {
        Id::Standard(sid) => CanFrame { id: sid.as_raw() as u32, data, dlc: dlc as u8, is_extended: false },
        Id::Extended(eid) => CanFrame { id: eid.as_raw(), data, dlc: dlc as u8, is_extended: true },
    }
}

#[embassy_executor::task]
pub async fn fdcan_bus_actor_task(
    mut rx: can::CanRx<'static>,
    mut tx: can::CanTx<'static>,
    bus: &'static mut fdCANbus,) -> ! 
{

    let cmd_ch = &bus.cmd_ch;

    let period = Duration::from_millis(1);
    let mut next = Instant::now() + period;

    let mut frames_to_send: [CanFrame; TX_FRAME_BUF_LEN] = [CanFrame::new(0, false); TX_FRAME_BUF_LEN];

    loop {
        // 等待三类事件：
        // 1) 到 1ms 控制点
        // 2) 收到 CAN 帧（收到即解析）
        // 3) 收到外部命令（设置目标值/重定位等）
        let tick_fut = Timer::at(next);
        let rx_fut = rx.read();
        let cmd_fut = cmd_ch.receive();

        // 用嵌套 select 实现三路等待：select( select(tick, rx), cmd )
        let event = embassy_futures::select::select(
            embassy_futures::select::select(tick_fut, rx_fut),
            cmd_fut,
        )
        .await;

        match event 
        {
            embassy_futures::select::Either::First(inner) => match inner 
            {
                embassy_futures::select::Either::First(_) => 
                {
                    // Tick
                    next += period;
                    while Instant::now() > next {next += period;}

                    // 1) update（与你 C++ 第一段一致）
                    for slot in bus.motors.iter_mut() 
                    {
                        if let Some(m_ref) = slot.as_mut() 
                        {
                            let motor: &mut dyn Motor_Base = &mut **m_ref;

                            let freq = motor.get_control_frequency();
                            let divider: u16 = if freq == 0 
                            {
                                1
                            } 
                            else 
                            {
                                (DEFAULT_CONTROL_HZ / freq).max(1)
                            };

                            if motor.get_control_cnt().wrapping_add(1) >= divider 
                            {
                                motor.update();
                            }
                        }
                    }

                    // 2) pack（与你 C++ 第二段一致）
                    let mut frame_cnt: usize = 0;
                    for slot in bus.motors.iter_mut() 
                    {
                        if frame_cnt >= frames_to_send.len() { break; } // 发送缓存已满，跳过剩余电机（极端情况）

                        let Some(m_ref) = slot.as_mut() else { continue };
                        let motor: &mut dyn Motor_Base = &mut **m_ref;

                        let freq = motor.get_control_frequency();
                        let divider: u16 = if freq == 0 
                        {
                            1
                        } 
                        else 
                        {
                            (DEFAULT_CONTROL_HZ / freq).max(1)
                        };

                        let due = if freq == DEFAULT_CONTROL_HZ 
                        {
                            true
                        } 
                        else 
                        {
                            motor.get_control_cnt().wrapping_add(1) >= divider
                        };

                        if !due 
                        {
                            motor.inc_control_cnt();
                            continue;
                        }

                        let wrote = motor.pack_command(&mut frames_to_send[frame_cnt..]);
                        frame_cnt = (frame_cnt + wrote).min(frames_to_send.len());
                        motor.reset_control_cnt();
                    }

                    // 3) send（await 发生在这里，但不持有 motor 的可变借用）
                    for i in 0..frame_cnt 
                    {
                        let f = &frames_to_send[i];
                        let len = (f.dlc as usize).min(8);
                        let payload = &f.data[..len];

                        let out = if f.is_extended 
                        {
                            can::Frame::new_extended(f.id, payload).unwrap()
                        } 
                        else 
                        {
                            can::Frame::new_standard(f.id as u16, payload).unwrap()
                        };

                        let _ = tx.write(&out).await;
                    }

                    // 清空发送缓存（避免下一轮误用旧数据）
                    for f in frames_to_send.iter_mut() 
                    {
                        *f = CanFrame::new(0, false);
                    }
                }

                embassy_futures::select::Either::Second(rx_res) => 
                {
                    // RX frame: 收到即解析
                    if let Ok(envelope) = rx_res 
                    {
                        let (rx_frame, _) = envelope.parts();
                        let frame = to_canframe(&rx_frame);

                        for slot in bus.motors.iter_mut() 
                        {
                            if let Some(m_ref) = slot.as_mut() 
                            {
                                let motor: &mut dyn Motor_Base = &mut **m_ref;
                                if motor.match_frame(&frame) 
                                {
                                    motor.update_feedback(&frame);
                                }
                            }
                        }
                    }
                }
            },

            embassy_futures::select::Either::Second(cmd) => 
            {
                // 外部命令：只修改电机目标/状态，不直接 await 发送（保持 tick 稳定）
                apply_bus_cmd(&mut bus.motors, cmd);
            }
        }
    }
}

fn apply_bus_cmd(motors: &mut [Option<&'static mut dyn Motor_Base>; MAX_MOTORS], cmd: BusCmd) 
{
    match cmd {
        BusCmd::Common(c) => apply_common_cmd(motors, c),
        BusCmd::Dji(d) => apply_dji_cmd(motors, d),
    }
}

fn apply_common_cmd(motors: &mut [Option<&'static mut dyn Motor_Base>; MAX_MOTORS], cmd: CommonCmd) 
{
    match cmd 
    {
        CommonCmd::SetTargetRpm { motor_id, rpm } => 
        {
            with_motor_by_id(motors, motor_id, |m| m.set_target_RPM(rpm));
        }

        CommonCmd::SetTargetAngle { motor_id, angle } => 
        {
            with_motor_by_id(motors, motor_id, |m| m.set_target_angle(angle));
        }

        CommonCmd::SetTargetCurrent { motor_id, current } => 
        {
            with_motor_by_id(motors, motor_id, |m| m.set_target_current(current));
        }

        CommonCmd::SetTargetTotalAngle {motor_id,total_angle,} => 
        {
            with_motor_by_id(motors, motor_id, |m| m.set_target_total_angle(total_angle));
        }
    }
}

fn apply_dji_cmd(motors: &mut [Option<&'static mut dyn Motor_Base>; MAX_MOTORS], cmd: DjiCmd) 
{
    match cmd 
    {
        DjiCmd::RelocateTotalAngle { motor_id, value } => 
        {
            with_motor_by_id(motors, motor_id, |m| m.relocate_total_angle(value));
        }
    }
}

fn with_motor_by_id(
    motors: &mut [Option<&'static mut dyn Motor_Base>; MAX_MOTORS],
    motor_id: u32,
    f: impl FnOnce(&mut dyn Motor_Base),) -> bool 
{
    for slot in motors.iter_mut() 
    {
        let Some(m_ref) = slot.as_mut() else { continue };
        let m: &mut dyn Motor_Base = &mut **m_ref;
        if m.get_motor_id() == motor_id 
        {
            f(m);
            return true;
        }
    }
    false
}