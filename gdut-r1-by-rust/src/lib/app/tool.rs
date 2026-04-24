

//将 传入的deg角度 归一化到 0-360度
pub fn normalize_deg_0_360(a: f32) -> f32 
{
    let mut r = a.rem_euclid(360.0);
    if r == -0.0 {r = 0.0;}
    r
}

pub fn normalize_deg_pm180(a: f32) -> f32 {
    let mut r = (a + 180.0).rem_euclid(360.0);
    if r == -0.0 {r = 0.0;}
    r - 180.0
}