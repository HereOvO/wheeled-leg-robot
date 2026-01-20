#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

//=================include===========================
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"
#include "jetson_uart.h"
#include "math.h"

//=================extern PV===========================
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;


extern osMutexId_t UART4mutex_id;      // 互斥量句柄
extern osMessageQueueId_t MotorQueueHandle;
// ================= 用户参数配置区(更换电机或者修改采样周期需要修改) =================

// 1. 电机参数 (根据你的图片)
#define MOTOR_BASE_PPR        7     // 霍尔传感器基础脉冲数 (电机尾部转一圈的物理脉冲)
#define MAX_MOTOR_PWM_PULSE_5_12  13999  // 电机控制的PWM最大值
#define MAX_MOTOR_PWM_PULSE_9  65535  // 电机控制的PWM最大值

#define PID_Control_Frq  100  // PID的控制频率,注意不能超过实际freertos中PID控制函数的运行频率
#define MOTOR_DEAD_ZONE_PULSE 150  //死区pulse
// 2. 减速比 (!!非常重要!!)
// N20电机通常都有减速箱，比如 1:30, 1:50, 1:100。
// 图片中并没有写具体减速比，你需要查看你的订单或者数一下：
// 如果你想要控制的是【减速后的输出轴】转速，这里必须填减速比(例如 30)。
// 如果你只关心电机尾部转速，这里填 1。
#define GEAR_RATIO            42.0f 

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

// ==================== 数据结构定义 =================

/* 电机状态 */
typedef struct {
		uint8_t  dev;
    float    speed_rpm;      // 当前转速 (RPM, 转/分钟)
    int8_t   direction;      // 当前转向 (1:正转, -1:反转, 0:静止)
    int16_t  encode_delta;   // 本次采样周期内的计数值增量
    int32_t  total_count;    // (可选) 累计总脉冲数，用于测距离
    uint16_t last_counter;   // 上一次读取的 TIM2->CNT 值
		uint8_t IsStatusChanged; //数据是否改变
} Motor_Status_t;

/* 电机PID控制参数 */
typedef struct {
    float Kp;           // 比例
    float Ki;           // 积分
    float Kd;           // 微分
    
    float prevError;    // 上次误差
    float integral;     // 积分累计
    
    // 用于微分项滤波的变量
    float prevDerivative;      // 上一次的微分值
    float filteredDerivative;  // 滤波后的微分值
} PID_Controller;






//=======================函数===========================
/* 设置指定电机的PWM参数 */ 
extern void SetMotorPWM(uint8_t motor_id, int8_t direction, uint32_t pulse);

/* 将执行rpm转换为pulse */
extern uint32_t RpmToPulse(float rpm_value);

/* 停止指定电机的PWM输出 */ 
extern void StopMotorPWM(uint8_t motor_id);

#endif

