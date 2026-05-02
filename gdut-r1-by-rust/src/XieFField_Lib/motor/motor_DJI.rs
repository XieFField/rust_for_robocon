#![allow(dead_code)] //允许未使用的代码 

use core::panic;

use super::motor_base::{MotorBaseData, Motor_Base};
use crate::XieFField_Lib::bsp::fdCANbus::CanFrame;
use crate::XieFField_Lib::bsp::encoder::{Encoder};
use crate::XieFField_Lib::app::pid::{PID_Incremental,PID_Position,PID_Param_Config};
use crate::XieFField_Lib::app::tool;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DJI_Motor_Type
{
    M3508,
    M2006,
    M6020,
}

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

const SEND_ID_LOW:u32 = 0x200;
const SEND_ID_HIGH:u32 = 0x1FF;
const SEND_ID_6020_LOW:u32 = 0x1FE;
const SEND_ID_6020_HIGH:u32 = 0x2FE;

pub struct  DJI_Motor {
    base: MotorBaseData,
    motor_type: DJI_Motor_Type,
    encoder: Encoder,
    pub angle_pid_time_psc: u16,
    pub angle_pid_time_cnt: u16,
    calc_total_angle: bool,
    calc_angle: bool,
    encoder_raw: u16,
}

impl DJI_Motor{
    fn new(motor_id: u32, motor_type: DJI_Motor_Type, gear_ratio: f32, 
            calc_total_angle: bool, calc_angle: bool) -> Self
    {
        if !calc_total_angle && calc_angle
        {
            panic!("[定义冲突]: calc_angle若想为true,则calc_total_angle必须为true");
        }

        DJI_Motor{
            base: MotorBaseData::new(motor_id, false, gear_ratio),
            motor_type,
            encoder: Encoder::new(8192), //假设编码器分辨率为8192，初始位置为0
            angle_pid_time_psc: 0,
            angle_pid_time_cnt: 0,
            calc_total_angle,
            calc_angle,
            encoder_raw: 0,
        }
    }
}

impl Motor_Base for DJI_Motor{
    #[allow(unused_variables)]
    fn update(&mut self) {}
    
    #[allow(unused_variables)]
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize {0}

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
                self.base_data_mut().angle = tool::normalize_deg_0_360(self.base_data().total_angle);
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


pub struct  DJI_Group{
    base_tx_id: u32,
    motors: [Option<&'static DJI_Motor>; 4],
    motor_count: u8,
    contains_gm6020: bool,
}

impl DJI_Group {
    fn new(base_tx_id: u32) -> Self 
    {
        DJI_Group {
            base_tx_id,
            motors: [None, None, None, None],
            motor_count: 0,
            contains_gm6020: false,
        }
    }

    pub fn add_motor(&mut self, motor:&'static DJI_Motor) -> bool
    {
        if self.motor_count >= 4
        {
            panic!("[添加失败]: DJI_Group{}已满员，无法添加更多电机", self.base_tx_id);
        }

        if motor.base_data().motor_id < 1 || motor.base_data().motor_id > 8
        {
            panic!("[添加失败]: DJI电机ID {} 不合法,必须在1到8之间", motor.base_data().motor_id);
        }

        if motor.motor_type == DJI_Motor_Type::M6020
        {
            if self.base_tx_id != SEND_ID_6020_LOW && self.base_tx_id != SEND_ID_6020_HIGH
            {
                panic!("[添加失败]: DJI M6020电机只能添加到base_tx_id为0x1FE或0x2FE的DJI_Group中");
            }
            else if self.base_tx_id == SEND_ID_6020_LOW && !(motor.base_data().motor_id >=1 && motor.base_data().motor_id <= 4)
            {
                panic!("[添加失败]: base_tx_id为0x1FE的DJI_Group只能添加ID在1到4之间的M6020电机");
            }
            else if self.base_tx_id == SEND_ID_6020_HIGH && !(motor.base_data().motor_id >=5 && motor.base_data().motor_id <= 7)
            {
                panic!("[添加失败]: base_tx_id为0x2FE的DJI_Group只能添加ID在5到7之间的M6020电机");
            }
        }
        else if  motor.motor_type == DJI_Motor_Type::M3508 || motor.motor_type == DJI_Motor_Type::M2006
        {
            if self.base_tx_id != SEND_ID_LOW && self.base_tx_id != SEND_ID_HIGH
            {
                panic!("[添加失败]: DJI M3508/M2006电机只能添加到base_tx_id为0x1FF或0x2FF的DJI_Group中");
            }
            else if self.base_tx_id == SEND_ID_LOW && !(motor.base_data().motor_id >=1 && motor.base_data().motor_id <= 4)
            {
                panic!("[添加失败]: base_tx_id为0x1FF的DJI_Group只能添加ID在1到4之间的M3508/M2006电机");
            }
            else if self.base_tx_id == SEND_ID_HIGH && !(motor.base_data().motor_id >=5 && motor.base_data().motor_id <= 8)
            {
                panic!("[添加失败]: base_tx_id为0x2FF的DJI_Group只能添加ID在5到8之间的M3508/M2006电机");
            }
        }
        else 
        {
            panic!("[添加失败]: 不支持的DJI电机类型");
        }

        if self.motor_count == 0
        {
            if motor.motor_type == DJI_Motor_Type::M6020
            {
                self.contains_gm6020 = true;
            }
            else 
            {
                self.contains_gm6020 = false;
            }
        }
        else 
        {
            if motor.motor_type == DJI_Motor_Type::M6020 && !self.contains_gm6020
            {
                panic!("[添加失败]: 已有非M6020电机,无法添加M6020电机");
            }
            else if motor.motor_type != DJI_Motor_Type::M6020 && self.contains_gm6020
            {
                panic!("[添加失败]: 已有M6020电机,无法添加非M6020电机");
            }
        }

        let slot = self.calc_slot(motor.base_data().motor_id, motor.motor_type);
        if self.motors[slot as usize].is_some() { return false; }//被占用

        if slot < 0 || slot >= 4 {return  false;}
        
        self.motors[slot as usize] = Some(motor);
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
    #[allow(unused_variables)]
    fn update(&mut self) {}
    
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize 
    {
        if out_frames.is_empty() || self.motor_count == 0 { return 0; }
        let frame = &mut out_frames[0];
        *frame = CanFrame::new(self.base_tx_id, false);
        frame.dlc = 8;
        for (i, motor_opt) in self.motors.iter().enumerate() 
        {
            if let Some(m) = motor_opt 
            {
                let current = real_to_raw_current(m.motor_type,m.get_target_current());
                frame.data[i * 2] = (current >> 8) as u8;
                frame.data[i * 2 + 1] = current as u8;
            }
        }
        1
    }

    #[allow(unused_variables)]
    fn update_feedback(&mut self, in_frame: &CanFrame) {}

    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data_mut(&mut self) -> &mut MotorBaseData 
    {
        panic!("[错误]: DJI_Group不可使用base_data_mut接口");
    }
    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data(&self) -> &MotorBaseData 
    {
        panic!("[错误]: DJI_Group不可使用base_data接口");
    }
}

pub enum DJI_Control_Mode
{
    Current,
    RPM,
    Angle,
    TotalAngle,
}

pub struct M3508{
    base: DJI_Motor,
    speed_pid: PID_Incremental,
    pos_pid: PID_Position,
    mode: DJI_Control_Mode,
    pos_ctrlcnt: u16,
}

const M3508_GEAR_RATIO: f32 = 3591.0/187.0; //3508的齿轮比，电机转一圈输出轴转3591/187圈

impl M3508 {
    fn new(motor_id: u32, 
           calc_total_angle: bool, calc_angle: bool) -> Self
    {
        M3508{
            base: DJI_Motor::new(motor_id, DJI_Motor_Type::M3508, M3508_GEAR_RATIO, calc_total_angle, calc_angle),

            speed_pid: PID_Incremental::new(PID_Param_Config::default(), 0.0, 0.001),
            pos_pid: PID_Position::new(PID_Param_Config::default(), false, 0.0, 0.01),

            mode: DJI_Control_Mode::Current,
            pos_ctrlcnt: 0,
        }
    }

    fn set_pos_pid_param(&mut self, param_config: PID_Param_Config)
    {
        self.pos_pid.param_config = param_config;
    }

    fn set_speed_pid_param(&mut self, param_config: PID_Param_Config)
    {
        self.speed_pid.param_config = param_config;
    }

    fn reset_motor_gear_ratio(&mut self, new_gear_ratio: f32)
    {
        if new_gear_ratio > 0.0
        {
            self.base.base_data_mut().gear_ratio = new_gear_ratio;
            self.base.base_data_mut().inv_gear_ratio = 1.0 / new_gear_ratio;
        }
    }
}

impl Motor_Base for M3508{
    #[allow(unused_variables)]
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize {0}

    fn update_feedback(&mut self, in_frame: &CanFrame) {self.base.update_feedback(in_frame);}

    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data(&self) -> &MotorBaseData {self.base.base_data()}
    #[doc(hidden)]
    #[allow(private_interfaces)]
    fn base_data_mut(&mut self) -> &mut MotorBaseData {self.base.base_data_mut()}

    fn get_RPM(&self) -> f32 {self.base.get_RPM()}
    fn get_current(&self) -> f32 {self.base.get_current()}
    fn get_angle(&self) -> f32 {self.base.get_angle()}
    fn get_total_angle(&self) -> f32 {self.base.get_total_angle()}

    fn set_target_RPM(&mut self, tar_RPM: f32) 
    {
        self.mode = DJI_Control_Mode::RPM;
        self.base.base_data_mut().target_rpm = tar_RPM;
        self.base.base_data_mut().target_angle = 0.0;
        self.base.base_data_mut().target_total_angle = 0.0;
    }

    fn set_target_current(&mut self, tar_current: f32) 
    {
        self.mode = DJI_Control_Mode::Current;
        self.base.base_data_mut().target_current = tar_current;
        self.base.base_data_mut().target_rpm = 0.0;
        self.base.base_data_mut().target_angle = 0.0;
        self.base.base_data_mut().target_total_angle = 0.0;
    }

    fn set_target_angle(&mut self, tar_angle: f32) 
    {
        self.mode = DJI_Control_Mode::Angle;
        self.base.base_data_mut().target_angle = tar_angle;
        self.base.base_data_mut().target_rpm = 0.0;
        self.base.base_data_mut().target_current = 0.0;
        self.base.base_data_mut().target_total_angle = 0.0;
    }

    fn set_target_total_angle(&mut self, tar_total_angle: f32) 
    {
        self.mode = DJI_Control_Mode::TotalAngle;
        self.base.base_data_mut().target_total_angle = tar_total_angle;
        self.base.base_data_mut().target_rpm = 0.0;
        self.base.base_data_mut().target_current = 0.0;
        self.base.base_data_mut().target_angle = 0.0;
    }

    fn update(&mut self) 
    {
        self.pos_ctrlcnt = self.pos_ctrlcnt.wrapping_add(1);
        match self.mode {
            DJI_Control_Mode::Current => 
            {
                //电流控制模式下不需要计算，直接发送目标电流即可
                return;
            },

            DJI_Control_Mode::RPM => 
            {
                self.base.base_data_mut().target_current = 
                    self.speed_pid.pid_calc(self.base.get_target_RPM(), self.base.get_RPM());
            },

            DJI_Control_Mode::Angle => 
            {
                if self.pos_ctrlcnt % 10 == 0 // 每10个控制周期更新一次位置PID
                {
                    self.base.base_data_mut().target_rpm = 
                        self.pos_pid.pid_calc(self.base.get_target_angle(), self.base.get_angle());
                }

                self.base.base_data_mut().target_current = 
                    self.speed_pid.pid_calc(self.base.base_data().target_rpm, self.base.get_RPM());
            },

            DJI_Control_Mode::TotalAngle => 
            {
                if self.pos_ctrlcnt % 10 == 0 // 每10个控制周期更新一次位置PID
                {
                    self.base.base_data_mut().target_rpm = 
                        self.pos_pid.pid_calc(self.base.get_target_total_angle(), self.base.get_total_angle());
                    
                }
                self.base.base_data_mut().target_current = 
                        self.speed_pid.pid_calc(self.base.base_data().target_rpm, self.base.get_RPM());
            },
        }
    }
}
