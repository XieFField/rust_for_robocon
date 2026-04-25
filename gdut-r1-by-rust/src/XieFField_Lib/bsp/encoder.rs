use crate::XieFField_Lib::app::tool::{AngleNormalize};
use crate::XieFField_Lib::app::tool::{normalize_deg_0_360, normalize_deg_pm180};
pub struct Encoder{ //基础数据
    angle:f32,          //单圈角度(0..360)
    total_angle:f32,    //连续总累计角度
    is_init:bool,       

    offset:u16,         //初始的raw值

    range:u16,

    round_cnt: i32,     //计圈
    last_angle: f32,    //上一帧的单圈角度(0..360)
    start_angle: f32,   //初始时刻的总角度

    precision_offset: f32, //因重置圈数而产生的累计偏置
    
    has_pending_relocate: bool, //若是在首帧反馈前调用了重定位函数，则在首帧反馈后进行重定位
    pending_relocate_total_angle: f32, //重定位时的总角度，重定位后将当前总角度设置为该值

}

impl Default for Encoder{ //默认8192线编码器
    fn default() -> Self 
    {
        Encoder::new(8192)
    }
}


impl Encoder{
    pub fn new(range: u16) ->Self
    {
        Encoder{
            angle: 0.0,
            total_angle: 0.0,
            is_init: false,

            offset: 0,

            range,

            round_cnt: 0,
            last_angle: 0.0,
            start_angle: 0.0,

            precision_offset: 0.0,

            has_pending_relocate: false,
            pending_relocate_total_angle: 0.0,
        }
    }

    pub fn angle(&self) -> f32{self.angle}
    pub fn total_angle(&self) -> f32{self.total_angle}
    
    pub fn angle_rad(&self) -> f32{self.angle.to_radians()}
    pub fn total_angle_rad(&self) -> f32{self.total_angle.to_radians()}

    pub fn update(&mut self, raw_value: u16)
    {
        let current_angle = raw_value as f32 / self.range as f32 * 360.0;

        if !self.is_init
        {
            self.offset = raw_value; //记录初始的raw 值
            self.start_angle = current_angle; // 记录初始角度
            self.last_angle = current_angle; 
            self.is_init = true;

            self.round_cnt = 0;
            self.precision_offset = 0.0;

            self.total_angle = normalize_deg_0_360(current_angle - self.start_angle);

            self.angle = normalize_deg_0_360(current_angle - self.start_angle);

            if self.has_pending_relocate
            {
                self.precision_offset = self.pending_relocate_total_angle;
                self.total_angle = self.pending_relocate_total_angle;
                self.angle = normalize_deg_0_360(self.total_angle);
                self.has_pending_relocate = false;
            }

            self.is_init = true;

            return;
        }

        let delta_angle = current_angle - self.last_angle;

        if delta_angle > 180.0 { self.round_cnt -= 1; }
        else if delta_angle < -180.0 { self.round_cnt += 1; }

        self.last_angle = current_angle; //updae

        let abs_total = self.round_cnt as f32 * 360.0 + current_angle;
        self.total_angle = (abs_total - self.start_angle) + self.precision_offset;
        self.angle = normalize_deg_0_360(self.total_angle);

        if self.round_cnt.abs() > 5000 
        {
            self.precision_offset += self.round_cnt as f32 * 360.0;
            self.round_cnt = 0;
        }
    }

    pub fn relocate_total_angle(&mut self, now_total_angle: f32)
    {
        if !self.is_init 
        {
            // 尚未收到首帧反馈时，先缓存目标，待初始化后立即应用
            self.has_pending_relocate = true;
            self.pending_relocate_total_angle = now_total_angle;
            self.total_angle = now_total_angle;
            self.angle = normalize_deg_0_360(self.total_angle);
            return;
        }

        // 当前计算出的总角度与目标总角度之间的差值，作为新的偏移量
        let current_calc = self.round_cnt as f32 * 360.0 + self.last_angle - self.start_angle;
        // 更新偏移量，使得当前计算的总角度调整为目标总角度
        self.precision_offset = now_total_angle - current_calc;
        self.total_angle = now_total_angle; 
        self.angle = normalize_deg_0_360(self.total_angle);
    }
}