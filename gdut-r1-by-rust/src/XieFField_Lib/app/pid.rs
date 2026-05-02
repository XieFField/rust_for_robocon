#![allow(dead_code)]

use embassy_time::{ Instant};
use crate::XieFField_Lib::app::tool::{Constrain};
pub struct PID_Param_Config{
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub output_limit: f32,
    pub integral_limit: f32, //积分限幅
    pub is_intergral_limit: bool, //是否启用积分限幅
    pub dead_zone: f32, //死区
    pub last_time: Option<Instant>, //上次计算的时间点
}

impl PID_Param_Config{
    pub fn new(kp: f32, ki: f32, kd: f32, 
        output_limit: f32, integral_limit: f32, is_intergral_limit: bool, dead_zone: f32) -> Self
    {
        PID_Param_Config{
            kp,
            ki,
            kd,
            output_limit,
            integral_limit,
            is_intergral_limit,
            dead_zone,
            last_time: None,
        }
    }
}

impl Default for PID_Param_Config{
    fn default() -> Self 
    {
        PID_Param_Config{
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            output_limit: 0.0,
            integral_limit: 0.0,
            is_intergral_limit: false,
            dead_zone: 0.0,
            last_time: None,
        }
    }
}

pub struct PID_Incremental{
    pub param_config: PID_Param_Config,

    output: f32,
    last_output: f32,

    error: f32,
    last_error: f32,
    last_last_error: f32,
    is_first_calc: bool, //是否第一次计算

    pub td_radio: f32, //td增量项的权重，0表示不开启td

    pub dt_default: f32, //默认dt

    td_v1: f32, //td计算的v1
    td_v2: f32, //td计算的v2
}


impl PID_Incremental{ //增量式PID
    pub fn new(param_config: PID_Param_Config,td_radio: f32,dt_default: f32,) -> Self 
    {
        PID_Incremental {
            param_config,
            output: 0.0,
            last_output: 0.0,
            error: 0.0,
            last_error: 0.0,
            last_last_error: 0.0,
            is_first_calc: true,
            td_radio,
            td_v1: 0.0,
            td_v2: 0.0,
            dt_default,
        }
    }

    fn calc_track_D(&mut self, target:f32, dt: f32)
    {
        let fh = -self.td_radio * self.td_radio * (self.td_v1 - target)
                - 2.0f32 *self.td_v2 * self.td_radio;
        
        self.td_v1 = self.td_v2 * dt;
        self.td_v2 = fh *dt;
    }


    pub fn pid_calc(&mut self, target: f32, feedback: f32) -> f32
    {
        let now = Instant::now();
        let mut dt = self.dt_default; //默认1ms
        if let Some(last) = self.param_config.last_time 
        {
            dt = (now - last).as_micros() as f32 / 1_000_000.0; //计算dt，单位秒
        }
        self.param_config.last_time = Some(now);

        if dt <= 0.0
        {
            dt = self.dt_default; //防止时间异常导致的dt为0或负数
        }

        let mut current_target = target;

        if self.td_radio > 0.0
        {
            self.calc_track_D(current_target, dt);
            current_target = self.td_v1;
        }

        //calc error
        self.error = current_target - feedback;

        if self.error.abs() < self.param_config.dead_zone
        {
            self.error = 0.0; //在死区内，误差视为0
        }


        if self.is_first_calc
        {
            self.last_error = 0.0;
            self.last_last_error = 0.0;
            self.is_first_calc = false;

            self.output = 0.0;
            self.last_output = 0.0;

            return self.output;
        }
        
        let pi_term = self.param_config.kp * self.error;

        let mut i_term = self.param_config.ki * self.error;
        i_term = i_term.constrain(-self.param_config.integral_limit, self.param_config.integral_limit);

        let d_term = self.param_config.kd * (self.error - 2.0f32 * self.last_error + self.last_last_error);

        self.output = self.last_output + pi_term + i_term + d_term;

        self.output = self.output.constrain(-self.param_config.output_limit, self.param_config.output_limit);

        self.output
    }
}


pub struct PID_Position{
    pub param_config: PID_Param_Config,

    output: f32,

    error: f32,
    last_error: f32,

    feedback_last: f32, //上次反馈值 用于计算微分项

    pub is_circular: bool, //唤醒模式下寻找最短路径，用于(-180..180)和(0..360)的输入

    is_first_calc: bool, //是否第一次计算
    last_dt: f32, //上次计算的dt
    last_time: Option<Instant>, //上次计算的时间点

    pub I_Separate: f32, //积分分离阈值， 为0表示不开启积分分离

    pub dt_default: f32, //默认dt
}

impl PID_Position{//位置式PID
    pub fn new(
        param_config: PID_Param_Config,
        is_circular: bool,  // 角度闭环必开：true
        I_Separate: f32,    // 积分分离阈值
        dt_default: f32,) -> Self 
    {
        PID_Position {
            param_config,
            output: 0.0,
            error: 0.0,
            last_error: 0.0,
            feedback_last: 0.0,
            is_circular,
            is_first_calc: true,
            last_dt: dt_default,
            last_time: None,
            dt_default,
            I_Separate,
        }
    }

    pub fn pid_calc(&mut self, target: f32, feedback: f32) -> f32
    {
        let now = Instant::now();
        let mut dt = self.dt_default;   


        
        if let Some(last) = self.param_config.last_time 
        {
            dt = (now - last).as_micros() as f32 / 1_000_000.0;
        }
        self.param_config.last_time = Some(now);

        if self.is_first_calc 
        {
            self.is_first_calc = false;

            dt = self.dt_default; //第一次计算，dt用默认值
            self.last_error = target - feedback;
            self.feedback_last = feedback;

            return 0.0; //第一次计算输出0
        }


        if dt <= 0.0 || dt > 0.1
        {
            dt = self.dt_default; //防止时间异常导致的dt为0或过大
        }

        self.error = target - feedback;

        if self.is_circular
        {
            while self.error > 180.0
            {
                self.error -= 360.0;
            }

            while self.error < -180.0
            {
                self.error += 360.0;
            }
        }

        if self.error.abs() < self.param_config.dead_zone
        {
            self.output = 0.0; //在死区内，直接输出0
            return self.output;
        }

        let p_term = self.param_config.kp * self.error;

        let mut i_term:f32;
        if self.error.abs() < self.I_Separate && self.I_Separate > 0.0
        {
            i_term = self.param_config.ki * (self.error + self.last_error) * dt * 0.5; //积分分离，在误差较大时不积分

            if self.param_config.is_intergral_limit
            {
                i_term = i_term.constrain(-self.param_config.integral_limit, self.param_config.integral_limit);
            }
        }
        else 
        {
            i_term = 0.0; 
        }

        //calc d_term 微分先行

        let mut d_term = 0.0f32;
        if self.is_circular
        {            
            let mut diff_feedback = feedback - self.feedback_last;
            //处理环形
            if diff_feedback > 180.0
            {
                diff_feedback -= 360.0;
            }
            else if diff_feedback < -180.0
            {
                diff_feedback += 360.0;
            }

            d_term = self.param_config.kd * diff_feedback / dt;
        }

        //update
        self.last_error = self.error;
        self.feedback_last = feedback;
        self.last_dt = dt;

        self.output = p_term + i_term - d_term; //位置式微分项是减的
        self.output = self.output.constrain(-self.param_config.output_limit, self.param_config.output_limit);
        self.output
    }
}