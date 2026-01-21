
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
float PID_MotorArr0[3]={60,8,0.5};  // Kp=1.5, Ki=0.1, Kd=0.05 (提高Kp和Ki，加快响应)
float PID_MotorArr1[3]={50,10,0.5};
float PID_MotorArr2[3]={230,15,0.03};
float PID_MotorArr3[3]={230,15,0.03};

/* 全局电机状态数组 */ 
Motor_Status_t all_motors[4] = {0};

// 用于电机PWM控制的变量
uint8_t motor_pwm_tim_map[4] = {5, 5, 12, 9}; 
uint8_t motor_encoder_tim_map[4] = {1, 2, 3, 8}; 

// 输出控制数组
uint32_t Motor_Control_Pulse[4]; // PWM脉冲值 (这里假设是int32或uint32，通常PWM寄存器是整型)
int8_t  Motor_Control_Dir[4];   // 输出方向

//Mutex句柄
extern osMutexId_t MotorStatusMutexHandle;
extern osMutexId_t MotorPIDOutPulseMutexHandle;

//=================函数区===========================

/* 将RPM值转换为脉冲值 */ 
//uint32_t RpmToPulse(float rpm_value)
//{
//    uint32_t max_pulse = 13999;  // 所有定时器的ARR值
//    float adjusted_rpm = fabsf(rpm_value);

//    // 使用公式：当RPM达到最大值时，脉冲宽度接近ARR值
//    // 最大RPM为381，则系数为 13999 / 381 = 36.74
//    uint32_t pulse = (uint32_t)(adjusted_rpm * 36.74f);

//    // 限制脉冲宽度不超过ARR值
//    if (pulse > max_pulse) pulse = max_pulse;

//    return pulse;
//}


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
    
        int8_t cDir;
        uint32_t ulPulse;
    

            while(1)
            {
                /* 读取闭环控制的结果 */
                   osMutexAcquire(MotorPIDOutPulseMutexHandle,portMAX_DELAY);
                
                   cDir=Motor_Control_Dir[motor_id];
                   ulPulse=Motor_Control_Pulse[motor_id];
                
                   osMutexRelease(MotorPIDOutPulseMutexHandle);
                
                
                /* 输出PWM */
               switch(motor_id)
               {  
                case 0:  // 内部电机0 (物理电机1) - 使用TIM5的通道1(正转)和通道2(反转)
                    // 停止当前所有通道输出
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

                    if (cDir == 1) {
                        // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ulPulse);
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
                    } else if (cDir == -1) {
                        // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, ulPulse);
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
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);

                    if (cDir == 1) {
                        // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ulPulse);
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
                    } else if (cDir == -1) {
                        // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, ulPulse);
                    } else {
                        // 停止：两通道都保持低电平
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
                    }

                    // 启动PWM输出
                    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
                    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
                    break;

                case 2:  // 内部电机2 (物理电机3) - 使用TIM5的通道3(正转)和通道4(反转)
                    // 停止当前所有通道输出
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

                    if (cDir == 1) {
                        // 正转：通道3输出PWM（占空比与速度成正比），通道4保持低电平（占空比为0%）
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, ulPulse);
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
                    } else if (cDir == -1) {
                        // 反转：通道3保持低电平（占空比为0%），通道4输出PWM（占空比与速度成正比）
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, ulPulse);
                    } else {
                        // 停止：两通道都保持低电平
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
                    }

                    // 启动PWM输出
                    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
                    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
                    break;

                case 3:  // 内部电机3 (物理电机4) - 使用TIM12的通道1(正转)和通道2(反转)
                    // 停止当前所有通道输出
                    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

                    if (cDir == 1) {
                        // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, ulPulse);
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
                    } else if (cDir == -1) {
                        // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, ulPulse);
                    } else {
                        // 停止：两通道都保持低电平
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
                    }

                    // 启动PWM输出
                    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
                    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
                    break;
                }
                    
            }
}

/* 电机速度监视任务函数 */
void MotorSpeedTask(void *argument)
{
	

    
       // extern void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm);
        /* USER CODE BEGIN MotorSpeedTask */
        /* Infinite loop */

			 // 定义TIM句柄数组
	  	TIM_HandleTypeDef* tim_handles[] = {&htim1, &htim2,&htim3,&htim8};//NULL只是用来占位的
		// 定义结构体实例,互斥访问时需要放到函数里
			Motor_Status_t motor_n20 = { ((Motor_Status_t*)argument)->dev,0,0,0,0,0,1};
			uint8_t Motor_id=motor_n20.dev;//(0~3)
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
						
                        //调试输出
//               if(Motor_id==3)
//               {
//                taskENTER_CRITICAL() ;
//                UART_SendInt16_WithLabel(&huart1,"last_counter", motor_n20.last_counter);
//                UART_SendInt16_WithLabel(&huart1,"current_cnt",current_cnt);
//                UART_SendInt16_WithLabel(&huart1,"delta",delta);
//                taskEXIT_CRITICAL();
//               }
               
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
                    const int16_t DIRECTION_THRESHOLD = 0; // 方向判断阈值，防止小幅度抖动
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
                    else if(delta==0){
//                            // 如果变化很小，保持原方向（除非速度接近0）
//                            if (fabsf(motor_n20.speed_rpm) < 1.0f) {
//                                    motor_n20.direction = 0; // 低速时可以改变方向
//                                    cnt = 0; // 重置计数器
//                                    HAL_GPIO_WritePin(DebugIO_GPIO_Port,DebugIO_Pin,GPIO_PIN_RESET);
//                            }
                            // 否则保持原方向以防止抖动
                        motor_n20.direction = 0;
                    }


                    
                    // 更新电机数据到综合姿态信息 根据当前任务的设备ID来确定更新哪个电机的数据,只更新转动方向和速度
                    // 使用带符号速度更新（正转为正，反转为负）
                    float signed_speed = motor_n20.speed_rpm;		

//                    if(Motor_id==3)//调试输出pass
//                    {
//                       taskENTER_CRITICAL() ;
//                       UART_SendInt16_WithLabel(&huart1,"direction", motor_n20.direction);
//                       //UART_SendInt16_WithLabel(&huart1,"signed_speed",signed_speed );
//                       taskEXIT_CRITICAL();  
//                    
//                    }
                     
                    UpdateSingleMotorDataWithSignedSpeed(motor_n20.dev, signed_speed);
                }
           
                
			//更新全局电机状态数组
            osMutexAcquire(MotorStatusMutexHandle,portMAX_DELAY);               
            all_motors[motor_n20.dev] = motor_n20;
            osMutexRelease(MotorStatusMutexHandle);
       // ================= C. 任务调度 =================
            // 使用 vTaskDelayUntil 保证采样的绝对周期性
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        

  /* USER CODE END MotorSpeedTask */
}



