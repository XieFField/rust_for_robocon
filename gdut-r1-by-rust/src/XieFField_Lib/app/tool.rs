pub trait Constrain {
    fn constrain(self, min: Self, max: Self) -> Self;
}

impl<T> Constrain for T
where 
    T: PartialOrd,
{
     fn constrain(self, min: Self, max: Self) -> Self 
     {
        if self < min 
        {
            min
        } 
        else if self > max 
        {
            max
        } 
        else 
        {
            self
        }
    }
}

pub trait AngleNormalize {
    /// 归一化到 0~360°
    fn normalize_deg_0_360(self) -> Self;
    /// 归一化到 ±180°
    fn normalize_deg_pm180(self) -> Self;
}

impl<T> AngleNormalize for T   //用于self
where 
    T: core::ops::Rem<Output = T>//支持取模
     + core::cmp::PartialOrd //可比大小
     + core::convert::From<f32> //可以从f32转换
     + Copy
     + core::ops::Add<Output = T>  //可相加
     + core::ops::Sub<Output = T>  //可相减
{
    fn normalize_deg_0_360(self) -> Self 
    {
        let _360 = T::from(360.0);
        let mut r = self % _360;
        if r < T::from(0.0) 
        {
            r = r + _360;
        }
        r

    }

    fn normalize_deg_pm180(self) -> Self 
    {
        let _360 = T::from(360.0);
        let _180 = T::from(180.0);
        let mut r = (self + _180) % _360;
        if r < T::from(0.0) 
        {
            r = r + _360;
        }
        r - _180
    }
}


pub fn normalize_deg_0_360<T>(a: T) -> T  //用于临时计算
where
    T: core::ops::Rem<Output = T>
     + core::ops::Add<Output = T>
     + core::cmp::PartialOrd
     + core::convert::From<f32>
     + Copy,
{
    let _360 = T::from(360.0);
    let mut r = a % _360;
    if r < T::from(0.0) 
    {
        r = r + _360;
    }
    r
}

pub fn normalize_deg_pm180<T>(a: T) -> T
where
    T: core::ops::Rem<Output = T>
     + core::ops::Add<Output = T>
     + core::ops::Sub<Output = T>
     + core::cmp::PartialOrd
     + core::convert::From<f32>
     + Copy,
{
    let _180 = T::from(180.0);
    let _360 = T::from(360.0);
    let mut r = (a + _180) % _360;
    if r < T::from(0.0) 
    {
        r = r + _360;
    }
    r - _180
}