//==================== include =========================
#include "ClosedLoopControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "math.h" // 用于fabs函数

//==================== extern PV ======================
/* 电机0到3对应使用的TIM */
extern uint8_t motor_pwm_tim_map[4] ;
/* 电机id */
extern uint8_t Motor_0;
extern uint8_t Motor_1;
extern uint8_t Motor_2;
extern uint8_t Motor_3;

/* 电机状态结构体 */
extern Motor_Status_t motor0_n20 ;
extern Motor_Status_t motor1_n20 ;
extern Motor_Status_t motor2_n20 ;
extern Motor_Status_t motor3_n20 ;

/* 电机状态数组 */
extern Motor_Status_t all_motors[4];


/* 信号量句柄 */
extern osMutexId_t MotorPIDInGoalMutexHandle;
extern osMutexId_t MotorPIDOutPulseMutexHandle;
//==================== self PV =========================

// ============================================================
// 1. 数据结构与外部变量声明
// ============================================================

// 外部定义的变量 (控制指令与PID参数)
extern int8_t  motor_cmd_direction[4]; // +1, -1, 0
extern float   motor_cmd_speed[4];     // 正数 float
extern float   PID_MotorArr0[3];       // P, I, D for Motor 0
extern float   PID_MotorArr1[3];
extern float   PID_MotorArr2[3];
extern float   PID_MotorArr3[3];

// 输出控制数组
extern uint32_t Motor_Control_Pulse[4]; // PWM脉冲值
extern int8_t   Motor_Control_Dir[4];   // 输出方向

//Mutex句柄
extern osMutexId_t MotorStatusMutexHandle;

// ============================================================
// 2. 宏定义配置
// ============================================================





// ============================================================
// 3. 内部PID状态结构体
// ============================================================
// 用于保存每个电机上一次的误差和积分累计值
typedef struct {
    float prev_error;
    float integral;
} PID_Internal_State_t;

static PID_Internal_State_t pid_states[4] = {0};

//==================== Function =========================

/*
*   功能: 基于FreeRTOS的任务，计算PID并更新电机控制PWM输出
*   输入: 无 (通过全局变量交互)
*   输出: 更新 Motor_Control_Pulse 和 Motor_Control_Dir
*/  
void Motor_PID_TaskFunc(void *pvParameters)
{
    
    
    // 1. 准备PID参数指针数组，方便循环调用
    float *pid_params_list[4] = {PID_MotorArr0, PID_MotorArr1, PID_MotorArr2, PID_MotorArr3};
    
    // 2. FreeRTOS 时间控制变量
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / PID_Control_Frq); // 计算Ticks
    
    // 初始化时间
    xLastWakeTime = xTaskGetTickCount();

    // 计算Dt (时间间隔，单位：秒)
    const float dt = 1.0f / (float)PID_Control_Frq;

    // 临时变量
    float target_speed_signed;
    float actual_speed_signed;
    float error;
    float p_term, i_term, d_term, output_signed;
    float kp, ki, kd;
    
    // 积分限幅值 (防止积分项单独过大导致响应迟滞)
    static float max_integral_val; 
    static uint8_t TIM;
    for(;;)
    {
        for(int i = 0; i < 4; i++)
        {
            TIM=motor_pwm_tim_map[i];
            if(TIM==9)
            {
                max_integral_val=MAX_MOTOR_PWM_PULSE_9;
            }
            else if(TIM==5 || TIM==12)
            {
                max_integral_val=MAX_MOTOR_PWM_PULSE_5_12;
            }
            
            // --- A. 获取参数 ---
            kp = pid_params_list[i][0];
            ki = pid_params_list[i][1];
            kd = pid_params_list[i][2];
            
            
            
            // --- B. 检查并计算目标速度 (带符号) ---
            // 结合方向和速度：正转为正值，反转为负值
            taskENTER_CRITICAL(); //进入临界区
            if (motor_cmd_direction[i] == 0) {
                target_speed_signed = 0.0f;
            } else {
                target_speed_signed = motor_cmd_speed[i] * (float)motor_cmd_direction[i];
            }
            taskEXIT_CRITICAL(); //出临界区
            
            
            // --- C. 计算当前实际速度 (带符号) ---
            // all_motors[i].direction 可能是 0, 1, -1 (停止, 正转, 反转)
            osMutexAcquire(MotorStatusMutexHandle,portMAX_DELAY);
            actual_speed_signed = all_motors[i].speed_rpm;
            osMutexRelease(MotorStatusMutexHandle);
            // --- D. 计算误差 ---
            error = target_speed_signed - actual_speed_signed;
            
           if(i==0)   //调试输出
                {
                    taskENTER_CRITICAL() ;
                    //UART_SendInt16_WithLabel(&huart1,"Motor_Control_Dir", cDir);
                    UART_SendFloat_WithLabel_Manual(&huart1,"erro", error );
                    UART_SendFloat_WithLabel_Manual(&huart1,"speed_rpm", all_motors[i].speed_rpm );
                    UART_SendFloat_WithLabel_Manual(&huart1,"actual_speed_signed", actual_speed_signed);
                    taskEXIT_CRITICAL();
                }
            
            // --- E. PID 计算 ---
            
            // 1. 比例项
            p_term = kp * error;

            // 2. 积分项 (带抗饱和 Anti-Windup)
            pid_states[i].integral += (error * dt);
            
            // 积分限幅
            if (pid_states[i].integral > max_integral_val) {
                pid_states[i].integral = max_integral_val;
            } else if (pid_states[i].integral < -max_integral_val) {
                pid_states[i].integral = -max_integral_val;
            }
            
            // (可选) 停车且误差极小时清空积分
            if (target_speed_signed == 0.0f && fabsf(error) < 1.0f) {
                 pid_states[i].integral = 0.0f;
            }

            i_term = ki * pid_states[i].integral;

            // 3. 微分项
            d_term = kd * (error - pid_states[i].prev_error) / dt;
            pid_states[i].prev_error = error; // 更新上一次误差

            // 4. 总输出 (纯PID计算值)
            output_signed = p_term + i_term + d_term;

            // --- E2. 【新增】死区电压补偿 (Dead Zone Compensation) ---
            // 目的：让PWM值跳过电机静摩擦区间
            if (output_signed > 0.001f) {
                output_signed += (float)MOTOR_DEAD_ZONE_PULSE;
            } else if (output_signed < -0.001f) {
                output_signed -= (float)MOTOR_DEAD_ZONE_PULSE;
            }

            // --- F. 输出限幅与赋值 ---
            
            // 计算绝对值作为PWM Pulse
            float output_abs = fabsf(output_signed);
            
            // 这里的逻辑：PID正值->正向，PID负值->反向
            int8_t out_dir = 0;
            if (output_signed > 0.001f) {
                out_dir = 1;
            } else if (output_signed < -0.001f) {
                out_dir = -1;
            } else {
                out_dir = 0;
            }

            // 总输出限幅 (防止PWM超过定时器周期)
            if (output_abs > (float)max_integral_val) {
                output_abs = (float)max_integral_val;
            }

            // --- G. 写入控制数组 ---
            osMutexAcquire(MotorPIDOutPulseMutexHandle,portMAX_DELAY);
            Motor_Control_Dir[i]   = out_dir;
            Motor_Control_Pulse[i] = (uint32_t)output_abs;
            osMutexRelease(MotorPIDOutPulseMutexHandle);
        }

        // --- H. 任务挂起，等待下一个周期 ---
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}