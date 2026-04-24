use crate::bsp::fdCANbus::CanFrame;

pub struct MotorBaseData{ //电机基础数据结构
    pub motor_id: u32,
    pub is_extended: bool,

    // 目标值
    pub target_rpm: f32,
    pub target_current: f32,
    pub target_angle: f32,
    pub target_total_angle: f32,

    // 实际反馈（输出轴）
    pub rpm: f32,
    pub current: f32,
    pub angle: f32,
    pub total_angle: f32,
    pub temperature: f32,

    pub gear_ratio: f32,
    pub inv_gear_ratio: f32,

    pub control_frequency: u16,
    pub control_cnt: u16,
}

impl MotorBaseData{
    pub fn new(motor_id: u32, is_extended: bool, gear_ratio: f32) -> Self
    {
        MotorBaseData{
            motor_id,
            is_extended,

            target_rpm: 0.0,
            target_current: 0.0,
            target_angle: 0.0,
            target_total_angle: 0.0,

            rpm: 0.0,
            current: 0.0,
            angle: 0.0,
            total_angle: 0.0,
            temperature: 0.0,

            gear_ratio,
            inv_gear_ratio: 1.0 / gear_ratio,

            control_frequency: 1000,
            control_cnt: 0,
        }
    }
}

pub trait Motor_Base{
    //设置目标值接口
    fn set_target_RPM(&mut self, tar_RPM: f32);
    fn set_target_current(&mut self, tar_current: f32);
    fn set_target_angle(&mut self, tar_angle: f32);
    fn set_target_total_angle(&mut self, tar_total_angle: f32);

    //获取实际反馈接口
    fn get_RPM(&self) -> f32;
    fn get_current(&self) -> f32;
    fn get_angle(&self) -> f32;
    fn get_total_angle(&self) -> f32;
    fn get_temperature(&self) -> f32;

    //更新 更新电机状态
    fn update(&mut self);

    //打包要发送的can帧 返回总计发送多少帧 一般情况下都是发一帧
    fn pack_command(&mut self, out_frames: &mut [CanFrame]) -> usize;

    //解析函数
    fn update_feedback(&mut self, in_frame: &CanFrame);

    fn match_frame(&self, in_frame: &CanFrame) -> bool; //判断输入的can帧是否是发给这个电机的

    fn reset_control_cnt(&mut self) { self.motor_data_mut().control_cnt = 0; }
    fn reset_control_frequency(&mut self, new_freq: u16) 
    { 
        if new_freq > 0 && newFreq % 100 == 0 && newfreq <= 1000
        {
            self.motor_data_mut().control_frequency = new_freq; 
        }
        else
        {
            self.motor_data_mut().control_frequency = 1000; //默认1000Hz
        }
        
    }

    //一些便捷访问
    fn motor_data_mut(&mut self) -> &mut MotorBaseData;

    fn motor_data(&self) -> &MotorBaseData;

    fn gear_ratio(&self) -> f32 { self.motor_data().gear_ratio }
    fn inv_gear_ratio(&self) -> f32 { self.motor_data().inv_gear_ratio }

    fn motor_id(&self) -> u32 { self.motor_data().motor_id }
    fn is_extended(&self) -> bool { self.motor_data().is_extended }
    fn control_frequency(&self) -> u16 { self.motor_data().control_frequency }
    fn inc_control_cnt(&mut self) { self.motor_data_mut().control_cnt += 1; }
}