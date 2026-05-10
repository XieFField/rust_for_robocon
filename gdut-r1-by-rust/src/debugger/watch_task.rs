// @file watch_task.rs
// @brief 上行发送和下行接收，并在此规定协议
// 上行协议：arm.target_height=500.000\n
//           写命令反馈：OK arm.target_height=600.000\n  ERR bad_path: not found\n
// 下行协议：set arm.target_height 600.0\n 格式：set + 空格 + path + 空格 + value + \n
// 通道分配： up0 用于rprintln! up1 用于上行协议 down0 用于下行协议
