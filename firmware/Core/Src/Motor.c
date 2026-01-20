
#include "motor.h"
//=================self PV==========================
/* 电机id */
uint8_t Motor_0=0x00;
uint8_t Motor_1=0x01;
uint8_t Motor_2=0x02;
uint8_t Motor_3=0x03;

/* 电机状态结构体 */
Motor_Status_t motor0_n20 = { 0,0,0,0,0,0,0};
Motor_Status_t motor1_n20 = { 1,0,0,0,0,0,0};
Motor_Status_t motor2_n20 = { 2,0,0,0,0,0,0};
Motor_Status_t motor3_n20 = { 3,0,0,0,0,0,0};

/* 电机PID参数 */
float PID_MotorArr0[5]={1.5,1,0.05};  // Kp=1.5, Ki=0.1, Kd=0.05 (提高Kp和Ki，加快响应)
float PID_MotorArr1[5]={1.5,1,0.05};
float PID_MotorArr2[5]={1.5,1,0.05};
float PID_MotorArr3[5]={1.5,1,0.05};

/* 全局电机状态数组 */ 
Motor_Status_t all_motors[4] = {0};

// 用于电机PWM控制的变量
uint8_t motor_pwm_tim_map[4] = {0, 1, 0, 2}; // 内部电机0,2使用TIM5, 内部电机1使用TIM9, 内部电机3使用TIM12

//=================函数区===========================

/* 将RPM值转换为脉冲值 */ 
uint32_t RpmToPulse(float rpm_value)
{
    uint32_t max_pulse = 13999;  // 所有定时器的ARR值
    float adjusted_rpm = fabsf(rpm_value);

    // 使用公式：当RPM达到最大值时，脉冲宽度接近ARR值
    // 最大RPM为381，则系数为 13999 / 381 = 36.74
    uint32_t pulse = (uint32_t)(adjusted_rpm * 36.74f);

    // 限制脉冲宽度不超过ARR值
    if (pulse > max_pulse) pulse = max_pulse;

    return pulse;
}


/* 设置指定电机的PWM参数 */ 
void SetMotorPWM(uint8_t motor_id, int8_t direction, uint32_t pulse)
{
    if (motor_id >= 4) return;  // 无效的电机ID

    switch(motor_id)
    {
        case 0:  // 内部电机0 (物理电机1) - 使用TIM5的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

            if (direction == 1) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
            } else if (direction == -1) {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
            } else {
                // 停止：两通道都保持低电平
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
            break;

        case 1:  // 内部电机1 (物理电机2) - 使用TIM9的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

            if (direction == 1) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            } else if (direction == -1) {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse);
            } else {
                // 停止：两通道都保持低电平
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
            break;

        case 2:  // 内部电机2 (物理电机3) - 使用TIM5的通道3(正转)和通道4(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);

            if (direction == 1) {
                // 正转：通道3输出PWM（占空比与速度成正比），通道4保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pulse);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
            } else if (direction == -1) {
                // 反转：通道3保持低电平（占空比为0%），通道4输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pulse);
            } else {
                // 停止：两通道都保持低电平
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
            break;

        case 3:  // 内部电机3 (物理电机4) - 使用TIM12的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

            if (direction == 1) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            } else if (direction == -1) {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse);
            } else {
                // 停止：两通道都保持低电平
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            break;
    }
}



/* 停止指定电机的PWM输出 */ 
void StopMotorPWM(uint8_t motor_id)
{
    if (motor_id >= 4) return;  // 无效的电机ID

    switch(motor_id)
    {
        case 0:  // 内部电机0 (物理电机1) - 使用TIM5的通道1和2
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
            break;

        case 1:  // 内部电机1 (物理电机2) - 使用TIM9的通道1和2
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
            break;

        case 2:  // 内部电机2 (物理电机3) - 使用TIM5的通道3和4
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
            break;

        case 3:  // 内部电机3 (物理电机4) - 使用TIM12的通道1和2
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
            break;
    }
}

/* 电机控制的任务函数 */
void MotorCtrl_TasK(void *argument)
{
    Motor_Status_t* PrvMotorState = (Motor_Status_t*)argument;
    uint8_t motor_id = PrvMotorState->dev;
    if (motor_id >= 4) return;

    /* PID 控制器结构体 */
    PID_Controller Controller = {0}; // 初始化清零
    
    // 加载 PID 参数
    float Kp, Ki, Kd;
    switch(motor_id) {
        case 0: Kp = PID_MotorArr0[0]; Ki = PID_MotorArr0[1]; Kd = PID_MotorArr0[2]; break;
        case 1: Kp = PID_MotorArr1[0]; Ki = PID_MotorArr1[1]; Kd = PID_MotorArr1[2]; break;
        case 2: Kp = PID_MotorArr2[0]; Ki = PID_MotorArr2[1]; Kd = PID_MotorArr2[2]; break;
        case 3: Kp = PID_MotorArr3[0]; Ki = PID_MotorArr3[1]; Kd = PID_MotorArr3[2]; break;
        default: Kp=0; Ki=0; Kd=0; break;
    }
    Controller.Kp = Kp; Controller.Ki = Ki; Controller.Kd = Kd;

    // 状态变量
    int8_t target_dir;
    float target_speed; 
    float current_speed;
    int8_t current_dir;

    // 时间控制变量
    const TickType_t xPIDPeriod = pdMS_TO_TICKS(1); // PID 1ms 周期
    const TickType_t xCmdPeriod = pdMS_TO_TICKS(10); // 命令更新 10ms 周期
    TickType_t xLastPIDWakeTime = xTaskGetTickCount();
    TickType_t xLastCmdUpdateTime = xTaskGetTickCount();

    // 物理参数
    const float dt = 0.001f; // 1ms
    // 前馈增益: 13999(MaxPWM) / 381(MaxRPM) ≈ 36.74
    // 稍微给小一点(36.0)，让PID的积分项去补足剩下的，这样不容易超调
    const float FF_GAIN = 36.0f; 
    const float MAX_I_TERM = 5000.0f; // 积分项最大贡献值 (PWM Pulse)

    // 立即获取一次命令
    GetMotorCommand(motor_id, &target_dir, &target_speed);

    for(;;)
    {
        // 1. 严格控制 1ms 周期
        vTaskDelayUntil(&xLastPIDWakeTime, xPIDPeriod);

        // 2. 检查是否需要更新命令 (10ms 一次)
        if ((xTaskGetTickCount() - xLastCmdUpdateTime) >= xCmdPeriod) {
            GetMotorCommand(motor_id, &target_dir, &target_speed);
            xLastCmdUpdateTime = xTaskGetTickCount();
        }

        // 3. 获取反馈
        current_speed = all_motors[motor_id].speed_rpm;
        current_dir = all_motors[motor_id].direction;

        // 4. 数据标准化 (带符号的速度)
        float real_target = 0.0f;
        if (target_dir == 1)       real_target = target_speed;
        else if (target_dir == -1) real_target = -target_speed;
        // else 0

        float real_current = (current_dir == -1) ? -current_speed : current_speed;

        // ================= PID 计算核心 =================
        
        // A. 计算误差
        float error = real_target - real_current;

        // B. 积分项 (标准位置式 PID)
        // 只有在目标不为0时才积分，或者你需要停车时保持保持力矩
        if (real_target != 0.0f) {
            Controller.integral += error * dt;
        } else {
            // 刹车时清除积分，防止再次启动时猛冲
            Controller.integral = 0.0f; 
        }

        // 计算积分输出贡献并限幅 (Anti-windup)
        // 限制的是 Ki * integral 的结果，而不是 integral 本身，这样更直观
        float i_term = Controller.Ki * Controller.integral;
				if(Controller.Ki)
				{
					if (i_term > MAX_I_TERM) 
						{
							i_term = MAX_I_TERM;
							Controller.integral = MAX_I_TERM / Controller.Ki; // 反推限制累加器
						} 
					else if (i_term < -MAX_I_TERM) 
					{
						i_term = -MAX_I_TERM;
						Controller.integral = -MAX_I_TERM / Controller.Ki;
					}
				}

        // C. 微分项 (带低通滤波)
        float raw_derivative = (error - Controller.prevError) / dt;
        float alpha = 0.7f; // 滤波系数 0~1, 越大越接近原始值(新值占控制的比例)
        float filtered_derivative = Controller.filteredDerivative * (1.0f - alpha) + raw_derivative * alpha;
        Controller.filteredDerivative = filtered_derivative;
        Controller.prevError = error;
        float d_term = Controller.Kd * filtered_derivative;
        
        // D. 前馈项 (Feedforward)
        // 直接根据目标速度给出一个基础 PWM，大大减轻 PID 的负担
        float ff_term = real_target * FF_GAIN;

        // E. 总输出 (直接相加，不要乘系数！)
        // Output = FF + P + I + D
        float output_val = ff_term + (Controller.Kp * error) + i_term + d_term;

        // ================= 特殊处理 =================

        // 1. 启动/死区补偿 (仅在 PID 输出不足以驱动电机时补偿)
        // 假设电机死区大概是 500-800 pulse
        if (fabsf(real_target) > 1.0f) {
            if (output_val > 0 && output_val < 800.0f) output_val = 800.0f;
            else if (output_val < 0 && output_val > -800.0f) output_val = -800.0f;
        }
        
        // 2. 停车逻辑
        if (real_target == 0.0f) {
             // 强力刹车逻辑：如果当前还有速度，PID会自动产生反向力矩(因为error!=0)
             // 但为了更稳，可以给一个死区，速度很低时直接切断
             if (fabsf(real_current) < 5.0f) {
                 output_val = 0.0f;
             }
        }

        // 3. 输出总限幅
        if (output_val > MAX_MOTOR_PWM_PULSE) output_val = MAX_MOTOR_PWM_PULSE;
        else if (output_val < -MAX_MOTOR_PWM_PULSE) output_val = -MAX_MOTOR_PWM_PULSE;

        // ================= 硬件执行 =================
        
        int32_t final_pulse = (int32_t)fabsf(output_val);
        int8_t final_dir = 0; // 0:stop, 1:fwd, -1:rev

        // 判定方向：根据 PID 计算出的总输出方向来决定，而不是仅根据目标方向
        // 这样可以实现“反向制动”
        if (output_val > 0.0f) final_dir = 1;
        else if (output_val < 0.0f) final_dir = -1;
        else final_dir = 0;

        // 避免频繁的正反转切换 (防抖)
        if (final_pulse < 100) final_pulse = 0; // 很小的 PWM 视为 0

        // 设置 PWM (假设 SetMotorPWM 内部处理了 Pulse)
        // 注意：如果是刹车(dir=0)，pulse 设为 0
        if (final_dir == 0) final_pulse = 0;
        SetMotorPWM(motor_id, final_dir, final_pulse);

        // 设置 GPIO 方向
        // 建议把这里的 GPIO 操作封装成一个函数，太长了
        GPIO_TypeDef* pPort1 = NULL; uint16_t Pin1 = 0;
        GPIO_TypeDef* pPort2 = NULL; uint16_t Pin2 = 0;

        switch(motor_id) {
            case 0: 
                pPort1 = MotorDirectionControl_IO1_1_GPIO_Port; Pin1 = MotorDirectionControl_IO1_1_Pin;
                pPort2 = MotorDirectionControl_IO1_2_GPIO_Port; Pin2 = MotorDirectionControl_IO1_2_Pin; break;
            case 1: 
                pPort1 = MotorDirectionControl_IO2_1_GPIO_Port; Pin1 = MotorDirectionControl_IO2_1_Pin;
                pPort2 = MotorDirectionControl_IO2_2_GPIO_Port; Pin2 = MotorDirectionControl_IO2_2_Pin; break;
            case 2: 
                pPort1 = MotorDirectionControl_IO3_1_GPIO_Port; Pin1 = MotorDirectionControl_IO3_1_Pin;
                pPort2 = MotorDirectionControl_IO3_2_GPIO_Port; Pin2 = MotorDirectionControl_IO3_2_Pin; break;
            case 3: 
                pPort1 = MotorDirectionControl_IO4_1_GPIO_Port; Pin1 = MotorDirectionControl_IO4_1_Pin;
                pPort2 = MotorDirectionControl_IO4_2_GPIO_Port; Pin2 = MotorDirectionControl_IO4_2_Pin; break;
        }

				
			if (osMutexAcquire(UART4mutex_id, osWaitForever) == osOK) {
					// === 临界区开始 ===
					
					float float_value = error;  // 你的浮点数
					char buffer[32];
					
					// 方法1：使用sprintf
					//int len = sprintf(buffer, "%.4f\r\n", float_value);  // 保留4位小数
				 
					
					// 方法2：支持负数
					int len = sprintf(buffer, "%+.4f\r\n", float_value);  // 带符号
					HAL_UART_Transmit(&huart4, (uint8_t*)buffer, len, HAL_MAX_DELAY);
					// 方法3：科学计数法
					// int len = sprintf(buffer, "%.2e\r\n", float_value);  // 如 3.14e+00
					
					// 方法4：自动选择格式
					// int len = sprintf(buffer, "%g\r\n", float_value);  // 自动选择f或e格式
					
					// 释放互斥量
					osMutexRelease(UART4mutex_id);
}
//        if (final_dir == 1) { // 正转
//            HAL_GPIO_WritePin(pPort1, Pin1, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(pPort2, Pin2, GPIO_PIN_RESET);
//        } else if (final_dir == -1) { // 反转
//            HAL_GPIO_WritePin(pPort1, Pin1, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(pPort2, Pin2, GPIO_PIN_SET);
//        } else { // 刹车/停止 (两个都低电平通常是自然滑行，如果都高或者是特定组合可能是刹车，依驱动器而定，这里按你原来的全低处理)
//            HAL_GPIO_WritePin(pPort1, Pin1, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(pPort2, Pin2, GPIO_PIN_RESET);
//        }
    }
}

/* 电机速度监视 */
void MotorSpeedTask(void *argument)
{
	
			extern void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm);
			/* USER CODE BEGIN MotorSpeedTask */
			/* Infinite loop */

			 // 定义TIM句柄数组
	  	TIM_HandleTypeDef* tim_handles[] = { NULL,&htim1, &htim2,&htim3,&htim8};//NULL只是用来占位的
		// 定义结构体实例,互斥访问时需要放到函数里
			Motor_Status_t motor_n20 = { ((Motor_Status_t*)argument)->dev,0,0,0,0,0,1};
			uint8_t Motor_id=motor_n20.dev;//(1~4)
			TIM_HandleTypeDef* tim_handles_self=tim_handles[Motor_id];

    // 初始化上一次计数器值为当前值，防止第一次计算出错
    motor_n20.last_counter = __HAL_TIM_GET_COUNTER(tim_handles_self);

    // 获取当前 Tick 时间，用于精确延时(即作为vTaskDelayUntil的延时起点,记录的是上一个周期理论唤醒时间,这个值会被vTaskDelayUntil更新),这个时间是从SystemTick里面读的
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(SPEED_SAMPLE_TIME_MS);  //将周期的单位转换为tick
		uint8_t cnt=0;
		int16_t delta =0;
    while (1)
    {
			
						motor_n20.IsStatusChanged=1;//默认电机状态改变
			
						int16_t last_delta=motor_n20.encode_delta;//记录上一次的电机增量
						
            // ================= A. 数据采集 =================
            // 根据Motor_id选择对应的TIM
            uint16_t current_cnt = __HAL_TIM_GET_COUNTER(tim_handles[Motor_id]);
        
            // ================= B. 核心解算 =================
            
            // 1. 计算增量 (Delta)
             delta = (int16_t)(current_cnt - motor_n20.last_counter);
						
						//更新必要的字段
						// 更新状态结构体
						motor_n20.encode_delta = delta;
						motor_n20.last_counter = current_cnt;
						motor_n20.total_count += delta; // 如果需要记录跑了多远

						if(last_delta==motor_n20.encode_delta){motor_n20.IsStatusChanged=0;}//如果增量为0,则电机状态未改变.不需要重复更新
			
						
							if(motor_n20.IsStatusChanged)
							{

									
									// 2. 计算 RPM (转速)
									float rpm = ((float)delta) * (60000.0f / SPEED_SAMPLE_TIME_MS) / PULSES_PER_REV_OUTPUT;
									motor_n20.speed_rpm = rpm;
									

								
									// 3. 判断方向 (添加防抖动处理)
									const int16_t DIRECTION_THRESHOLD = 2; // 方向判断阈值，防止小幅度抖动
									if (delta > DIRECTION_THRESHOLD) {
											motor_n20.direction = 1;
											cnt++;
											if(cnt>5)HAL_GPIO_WritePin(DebugIO_GPIO_Port,DebugIO_Pin,GPIO_PIN_SET);
									}
									else if (delta < -DIRECTION_THRESHOLD) {
											motor_n20.direction = -1;
											cnt = 0; // 重置计数器
											HAL_GPIO_WritePin(DebugIO_GPIO_Port,DebugIO_Pin,GPIO_PIN_RESET);
									}
									else {
											// 如果变化很小，保持原方向（除非速度接近0）
											if (fabsf(motor_n20.speed_rpm) < 1.0f) {
													motor_n20.direction = 0; // 低速时可以改变方向
													cnt = 0; // 重置计数器
													HAL_GPIO_WritePin(DebugIO_GPIO_Port,DebugIO_Pin,GPIO_PIN_RESET);
											}
											// 否则保持原方向以防止抖动
									}
									 
									/* 将结构体写入队列,这个队列只被UART1使用,现在UART1已经被UART4取代 */
									//	osMessageQueuePut(MotorQueueHandle, &motor_n20, NULL, osWaitForever);
									
									
									// 更新电机数据到综合姿态信息 根据当前任务的设备ID来确定更新哪个电机的数据,只更新转动方向和速度
									// 使用带符号速度更新（正转为正，反转为负）
									float signed_speed = motor_n20.speed_rpm;									
									UpdateSingleMotorDataWithSignedSpeed(motor_n20.dev, signed_speed);
							}
           
							//更新全局电机状态数组
            all_motors[motor_n20.dev - 1] = motor_n20;
          
       // ================= C. 任务调度 =================
            // 使用 vTaskDelayUntil 保证采样的绝对周期性
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        

  /* USER CODE END MotorSpeedTask */
}

