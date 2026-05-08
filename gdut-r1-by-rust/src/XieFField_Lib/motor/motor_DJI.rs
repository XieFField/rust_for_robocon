#![allow(dead_code)] //允许未使用的代码 

use core::cell::RefCell;

use super::motor_base::{MotorBaseData, Motor_Base};
use crate::XieFField_Lib::bsp::bsp_fdCANbus::CanFrame;
use crate::XieFField_Lib::bsp::bsp_encoder::{Encoder};
use crate::XieFField_Lib::app::app_pid::{PID_Incremental,PID_Position,PID_Param_Config};
use crate::XieFField_Lib::app::app_tool;

/**
 * @attention: DJI_Motor需要注册到DJI_Group，由DJI_Group进行统一的CAN帧打包和反馈更新
 */


#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DJI_Motor_Type
{
    M3508,
    M2006,
    M6020,
}

pub enum DJI_Control_Mode
{
    Current,
    RPM,
    Angle,
    TotalAngle,
}

pub const M3508_GEAR_RATIO: f32 = 3591.0/187.0; //3508的齿轮比，电机转一圈输出轴转3591/187圈

const rawtoreal_M3508: f32 = 20000.0 / 16384.0; //电流raw和真实电流(ma)转换系数
const rawtoreal_M6020: f32 = 3000.0 / 16384.0; //电流raw和真实电流(ma)转换系数

pub fn raw_to_real_current(mttype: DJI_Motor_Type, raw: i16) -> f32
{
    match mttype {
        DJI_Motor_Type::M3508 => raw as f32 * rawtoreal_M3508,
        DJI_Motor_Type::M2006 => raw as f32 , // 1: 1
        DJI_Motor_Type::M6020 => raw as f32 * rawtoreal_M6020,
    }
}

pub fn real_to_raw_current(mttype: DJI_Motor_Type, real: f32) -> i16
{
    match mttype {
        DJI_Motor_Type::M3508 => (real / rawtoreal_M3508) as i16,
        DJI_Motor_Type::M2006 => real as i16, // 1: 1
        DJI_Motor_Type::M6020 => (real / rawtoreal_M6020) as i16,
    }
}

pub const SEND_ID_LOW:u32 = 0x200;
pub const SEND_ID_HIGH:u32 = 0x1FF;
pub const SEND_ID_6020_LOW:u32 = 0x1FE;
pub const SEND_ID_6020_HIGH:u32 = 0x2FE;

pub struct  DJI_Motor {
    base: MotorBaseData,
    motor_type: DJI_Motor_Type,
    encoder: Encoder,
    pub angle_pid_time_psc: u16,
    pub angle_pid_time_cnt: u16,
    calc_total_angle: bool,
    calc_angle: bool,
    encoder_raw: u16,

    speed_pid: Option<PID_Incremental>,
    pos_pid: Option<PID_Position>,
    control_mode: DJI_Control_Mode,
    pos_ctrlcnt: Option<u16>,
    pos_ctrl_freq: Option<u16>,
}

impl DJI_Motor{
    pub fn new_m3508(motor_id: u32, calc_total_angle: bool, calc_angle: bool) -> Self
    {
        if !calc_total_angle && calc_angle
        {
            panic!("[定义冲突]: calc_angle若想为true,则calc_total_angle必须为true");
        }

        DJI_Motor{
            base: MotorBaseData::new(motor_id, false, M3508_GEAR_RATIO),
            motor_type: DJI_Motor_Type::M3508,
            encoder: Encoder::new(8192), //假设编码器分辨率为8192，初始位置为0
            angle_pid_time_psc: 0,
            angle_pid_time_cnt: 0,
            calc_total_angle,
            calc_angle,
            encoder_raw: 0,
            speed_pid: None,
            pos_pid: None,
            control_mode: DJI_Control_Mode::Current,
            pos_ctrlcnt: None,
            pos_ctrl_freq: None,
        }
    }

    pub fn init_speed_pid(&mut self, param_config: PID_Param_Config)
    {
        self.speed_pid = Some(PID_Incremental::new(param_config, 0.0, 1.0 / (self.base.control_frequency as f32)));
    }

    pub fn init_pos_pid(&mut self, param_config: PID_Param_Config, is_circular: bool, I_Separate: f32)
    {
        self.pos_pid = Some(PID_Position::new(param_config, is_circular, I_Separate, 1.0 / (self.pos_ctrl_freq.unwrap_or(100) as f32)));
    }
}

impl Motor_Base for DJI_Motor{
    #[allow(unused_variables)]
    fn update(&mut self) 
    {
        if self.speed_pid.is_none() { return; } //没有速度pid，无法进行控制计算

        let freq = self.pos_ctrl_freq.unwrap_or(100);
        let divider = (1000 / freq).max(1); // 防止除零
        let mut cnt = self.pos_ctrlcnt.unwrap_or(0);
        cnt = cnt.wrapping_add(1);
        let do_pos = cnt % divider == 0;
        if do_pos { cnt = 0;}
        self.pos_ctrlcnt = Some(cnt);

        match self.control_mode 
        {
            DJI_Control_Mode::Current => {},

            DJI_Control_Mode::RPM => 
            {
                let target_rpm = self.base_data().target_rpm;
                let rpm = self.base_data().rpm;

                self.base_data_mut().target_current = 
                    self.speed_pid.as_mut().unwrap().pid_calc(target_rpm, rpm);
            },

            DJI_Control_Mode::Angle =>
            {
                if do_pos
                {
                    let target_angle = self.base_data().target_angle;
                    let angle = self.base_data().angle;
                    self.base_data_mut().target_rpm = 
                        self.pos_pid.as_mut().unwrap().pid_calc(target_angle, angle);
                }
                let target_rpm = self.base_data().target_rpm;
                let rpm = self.base_data().rpm;
                self.base_data_mut().target_current = 
                    self.speed_pid.as_mut().unwrap().pid_calc(target_rpm, rpm);
            }

            DJI_Control_Mode::TotalAngle =>
            {
                if do_pos
                {
                    let target_total_angle = self.base_data().target_total_angle;
                    let total_angle = self.base_data().total_angle;
                    self.base_data_mut().target_rpm = 
                        self.pos_pid.as_mut().unwrap().pid_calc(target_total_angle, total_angle);
                }
                let target_rpm = self.base_data().target_rpm;
                let rpm = self.base_data().rpm;
                self.base_data_mut().target_current = 
                    self.speed_pid.as_mut().unwrap().pid_calc(target_rpm, rpm);
            }
        }
    }
    
    fn set_target_current(&mut self, tar_current: f32) 
    {
        self.control_mode = DJI_Control_Mode::Current;
        self.base_data_mut().target_current = tar_current;
        self.base_data_mut().target_rpm = 0.0;
        self.base_data_mut().target_angle = 0.0;
        self.base_data_mut().target_total_angle = 0.0;    
    }

    fn set_target_RPM(&mut self, tar_RPM: f32) 
    {
        if self.speed_pid.is_none()
        {
            panic!("[错误]: speed_pid未初始化,无法使用set_target_RPM接口");
        }

        self.control_mode = DJI_Control_Mode::RPM;
        self.base_data_mut().target_rpm = tar_RPM;
        self.base_data_mut().target_angle = 0.0;
        self.base_data_mut().target_total_angle = 0.0;
    }

    fn set_target_angle(&mut self, tar_angle: f32) 
    {
        if self.pos_pid.is_none()
        {
            panic!("[错误]: pos_pid未初始化,无法使用set_target_angle接口");
        }

        if !self.calc_angle || !self.calc_total_angle
        {
            panic!("[错误]: calc_angle和calc_total_angle必须至少有一个为true,才能使用set_target_angle接口");
        }

        self.control_mode = DJI_Control_Mode::Angle;
        self.base_data_mut().target_angle = tar_angle;
        self.base_data_mut().target_total_angle = 0.0;
    }

    fn set_target_total_angle(&mut self, tar_total_angle: f32) 
    {
        if self.pos_pid.is_none()
        {
            panic!("[错误]: pos_pid未初始化,无法使用set_target_total_angle接口");
        }

        if !self.calc_total_angle
        {
            panic!("[错误]: calc_total_angle必须为true,才能使用set_target_total_angle接口");
        }

        self.control_mode = DJI_Control_Mode::TotalAngle;
        self.base_data_mut().target_total_angle = tar_total_angle;
        self.base_data_mut().target_angle = 0.0;    
    }

    #[allow(unused_variables)]
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize {0} //交给DJI_Group统一打包

    fn update_feedback(&mut self, in_frame: &CanFrame) 
    {
        let data = &in_frame.data;

        let encoder_raw = ((data[0] as u16) << 8) | (data[1] as u16);
        let rpm_raw = ((data[2] as i16) << 8) | (data[3] as i16);
        let current_raw = ((data[4] as i16) << 8) | (data[5] as i16);
        let temperature_raw = data[6] as i8; 
        
        //数值转换
        self.base_data_mut().rpm = rpm_raw as f32 * self.base_data().inv_gear_ratio;
        self.base_data_mut().current = raw_to_real_current(self.motor_type, current_raw);
        self.base_data_mut().temperature = temperature_raw as f32;

        if self.calc_total_angle
        {
            self.encoder_raw = encoder_raw;
            self.encoder.update(self.encoder_raw);
            self.base_data_mut().total_angle = self.encoder.total_angle() * self.base_data().gear_ratio;

            if self.calc_angle
            {
                self.base_data_mut().angle = app_tool::normalize_deg_0_360(self.base_data().total_angle);
            }
        }
    }

    fn match_frame(&self, in_frame: &CanFrame) -> bool 
    {
        if in_frame.is_extended
        {
            return false; //不处理扩展帧
        }

        if self.motor_type == DJI_Motor_Type::M6020
        {
            if in_frame.id < (0x204 + 1) || in_frame.id > (0x204 + 7)
            {
                return false;//非法id
            }
            else if in_frame.id == (0x204 + 1) && in_frame.id <= (0x204 + 7)
            {
                return true;//匹配成功
            }
            else
            {
                return false;//不匹配
            }

        }
        else if self.motor_type == DJI_Motor_Type::M3508 || self.motor_type == DJI_Motor_Type::M2006
        {
            if in_frame.id < (0x200 + 1) || in_frame.id > (0x200 + 7)
            {
                return false;//非法id
            }
            else if in_frame.id == (0x200 + 1) && in_frame.id <= (0x200 + 7)
            {
                return true;//匹配成功
            }
            else
            {
                return false;//不匹配
            }
        }
        else
        {
            return false; //类型匹配失败
        }
    }

    fn get_RPM(&self) -> f32 { self.base_data().rpm }
    fn get_current(&self) -> f32 { self.base_data().current }
    fn get_angle(&self) -> f32 { self.base_data().angle }
    fn get_total_angle(&self) -> f32 { self.base_data().total_angle }

    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data_mut(&mut self) -> &mut MotorBaseData {&mut self.base}
    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data(&self) -> &MotorBaseData {&self.base}

}


pub struct DJI_Group{
    base_tx_id: u32,
    /// &RefCell 引用: 电机本体注册在 bus 里 (通过 MotorCellWrapper),
    /// Group 只在 pack 时 borrow() 读取 target_current
    motors: [Option<&'static RefCell<DJI_Motor>>; 4],
    motor_count: u8,
    contains_gm6020: bool,
}

impl DJI_Group {
    pub fn new(base_tx_id: u32) -> Self
    {
        DJI_Group {
            base_tx_id,
            motors: [None, None, None, None],
            motor_count: 0,
            contains_gm6020: false,
        }
    }

    pub fn add_motor(&mut self, motor_cell: &'static RefCell<DJI_Motor>) -> bool
    {
        if self.motor_count >= 4
        {
            panic!("[添加失败]: DJI_Group{}已满员，无法添加更多电机", self.base_tx_id);
        }

        let motor = motor_cell.borrow();
        let mid = motor.base_data().motor_id;
        let mtype = motor.motor_type;

        if mid < 1 || mid > 8
        {
            panic!("[添加失败]: DJI电机ID {} 不合法,必须在1到8之间", mid);
        }

        if mtype == DJI_Motor_Type::M6020
        {
            if self.base_tx_id != SEND_ID_6020_LOW && self.base_tx_id != SEND_ID_6020_HIGH
            {
                panic!("[添加失败]: DJI M6020电机只能添加到base_tx_id为0x1FE或0x2FE的DJI_Group中");
            }
            else if self.base_tx_id == SEND_ID_6020_LOW && !(mid >=1 && mid <= 4)
            {
                panic!("[添加失败]: base_tx_id为0x1FE的DJI_Group只能添加ID在1到4之间的M6020电机");
            }
            else if self.base_tx_id == SEND_ID_6020_HIGH && !(mid >=5 && mid <= 7)
            {
                panic!("[添加失败]: base_tx_id为0x2FE的DJI_Group只能添加ID在5到7之间的M6020电机");
            }
        }
        else if  mtype == DJI_Motor_Type::M3508 || mtype == DJI_Motor_Type::M2006
        {
            if self.base_tx_id != SEND_ID_LOW && self.base_tx_id != SEND_ID_HIGH
            {
                panic!("[添加失败]: DJI M3508/M2006电机只能添加到base_tx_id为0x1FF或0x2FF的DJI_Group中");
            }
            else if self.base_tx_id == SEND_ID_LOW && !(mid >=1 && mid <= 4)
            {
                panic!("[添加失败]: base_tx_id为0x1FF的DJI_Group只能添加ID在1到4之间的M3508/M2006电机");
            }
            else if self.base_tx_id == SEND_ID_HIGH && !(mid >=5 && mid <= 8)
            {
                panic!("[添加失败]: base_tx_id为0x2FF的DJI_Group只能添加ID在5到8之间的M3508/M2006电机");
            }
        }
        else
        {
            panic!("[添加失败]: 不支持的DJI电机类型");
        }

        // 检查同类型一致性
        if self.motor_count == 0
        {
            self.contains_gm6020 = mtype == DJI_Motor_Type::M6020;
        }
        else if mtype == DJI_Motor_Type::M6020 && !self.contains_gm6020
        {
            panic!("[添加失败]: 已有非M6020电机,无法添加M6020电机");
        }
        else if mtype != DJI_Motor_Type::M6020 && self.contains_gm6020
        {
            panic!("[添加失败]: 已有M6020电机,无法添加非M6020电机");
        }

        drop(motor); // 释放 borrow, 后面的操作不需要

        let slot = self.calc_slot(mid, mtype);
        if slot < 0 || slot >= 4 { return false; }
        if self.motors[slot as usize].is_some() { return false; }

        self.motors[slot as usize] = Some(motor_cell);
        self.motor_count = self.motor_count.wrapping_add(1);
        true
    }

    fn calc_slot(&self, mid: u32, mtype: DJI_Motor_Type) -> isize
    {
        match mtype
        {
            DJI_Motor_Type::M6020 =>
            {
                if self.base_tx_id == SEND_ID_6020_LOW && (1..=4).contains(&mid) { (mid - 1) as isize }
                else if self.base_tx_id == SEND_ID_6020_HIGH && (5..=7).contains(&mid) { (mid - 5) as isize }
                else { -1 }
            }
            _ =>
            {
                if self.base_tx_id == SEND_ID_LOW && (1..=4).contains(&mid) { (mid - 1) as isize }
                else if self.base_tx_id == SEND_ID_HIGH && (5..=8).contains(&mid) { (mid - 5) as isize }
                else { -1 }
            }
        }
    }
}

impl Motor_Base for DJI_Group{
    // ── 不做 update/feedback/match: 由注册在 bus 里的单个 DJI_Motor 负责 ──
    fn update(&mut self) {}
    fn update_feedback(&mut self, _in_frame: &CanFrame) {}
    fn match_frame(&self, _in_frame: &CanFrame) -> bool { false }

    // ── 只负责：把 4 个电机的 target_current 打包进 1 帧 CAN ──
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize
    {
        if out_frames.is_empty() || self.motor_count == 0 { return 0; }
        let frame = &mut out_frames[0];
        *frame = CanFrame::new(self.base_tx_id, false);
        frame.dlc = 8;
        for (i, cell_opt) in self.motors.iter().enumerate()
        {
            if let Some(cell) = cell_opt
            {
                let motor = cell.borrow();
                let current = real_to_raw_current(motor.motor_type, motor.get_target_current());
                frame.data[i * 2] = (current >> 8) as u8;
                frame.data[i * 2 + 1] = current as u8;
            }
        }
        1
    }

    // ── 控制频率: Group 始终 1kHz 全速 ──
    fn get_control_frequency(&self) -> u16 { 1000 }
    fn get_control_cnt(&self) -> u16 { 0 }
    fn inc_control_cnt(&mut self) {}
    fn reset_control_cnt(&mut self) {}

    fn get_motor_id(&self) -> u32 { u32::MAX }

    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data_mut(&mut self) -> &mut MotorBaseData
    {
        panic!("[错误]: DJI_Group不支持base_data_mut接口");
    }
    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data(&self) -> &MotorBaseData
    {
        panic!("[错误]: DJI_Group不支持base_data接口");
    }
}




