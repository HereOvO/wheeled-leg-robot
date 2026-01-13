#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include "main.h" // 包含 HAL 库定义

// ================= 用户函数 =================


// ================= 用户参数配置区 =================

// 1. 电机参数 (根据你的图片)
#define MOTOR_BASE_PPR        7     // 霍尔传感器基础脉冲数 (电机尾部转一圈的物理脉冲)

// 2. 减速比 (!!非常重要!!)
// N20电机通常都有减速箱，比如 1:30, 1:50, 1:100。
// 图片中并没有写具体减速比，你需要查看你的订单或者数一下：
// 如果你想要控制的是【减速后的输出轴】转速，这里必须填减速比(例如 30)。
// 如果你只关心电机尾部转速，这里填 1。
#define GEAR_RATIO            30.0f 

// 3. 采样周期 (毫秒)
// 周期越短响应越快，但速度值跳动可能变大；周期越长速度越平滑，但有延迟。
// 10ms - 50ms 是常用范围。
//一圈总脉冲=基础脉冲×定时器倍频×减速比
//这里采样周期要合理设置,
#define SPEED_SAMPLE_TIME_MS  20    

// ================= 自动计算常量 =================

// 编码器模式 TI1 and TI2 代表 4 倍频
#define ENCODER_MULTIPLIER    4.0f

// 减速后的输出轴转一圈，定时器会计数的总数值
// 公式：7 * 4 * 30 = 840 (假设减速比30)
#define PULSES_PER_REV_OUTPUT (MOTOR_BASE_PPR * ENCODER_MULTIPLIER * GEAR_RATIO)

// ================= 数据结构定义 =================

typedef struct {
		uint8_t  dev;
    float    speed_rpm;      // 当前转速 (RPM, 转/分钟)
    int8_t   direction;      // 当前转向 (1:正转, -1:反转, 0:静止)
    int16_t  encode_delta;   // 本次采样周期内的计数值增量
    int32_t  total_count;    // (可选) 累计总脉冲数，用于测距离
    uint16_t last_counter;   // 上一次读取的 TIM2->CNT 值
} Motor_Status_t;

// 声明全局变量供其他文件调用
extern Motor_Status_t motor_n20;

#endif
