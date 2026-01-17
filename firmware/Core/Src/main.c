/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_mpu6050.h"
#include "pca9685.h"
#include "jetson_uart.h"
#include "Motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* Definitions for gpiotask */
osThreadId_t gpiotaskHandle;
const osThreadAttr_t gpiotask_attributes = {
  .name = "gpiotask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart_task */
osThreadId_t uart_taskHandle;
const osThreadAttr_t uart_task_attributes = {
  .name = "uart_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_task */
osThreadId_t imu_taskHandle;
const osThreadAttr_t imu_task_attributes = {
  .name = "imu_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pca9685task */
osThreadId_t pca9685taskHandle;
const osThreadAttr_t pca9685task_attributes = {
  .name = "pca9685task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for jetson_task */
osThreadId_t jetson_taskHandle;
const osThreadAttr_t jetson_task_attributes = {
  .name = "jetson_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask1 */
osThreadId_t MotorTask1Handle;
const osThreadAttr_t MotorTask1_attributes = {
  .name = "MotorTask1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask2 */
osThreadId_t MotorTask2Handle;
const osThreadAttr_t MotorTask2_attributes = {
  .name = "MotorTask2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask3 */
osThreadId_t MotorTask3Handle;
const osThreadAttr_t MotorTask3_attributes = {
  .name = "MotorTask3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask4 */
osThreadId_t MotorTask4Handle;
const osThreadAttr_t MotorTask4_attributes = {
  .name = "MotorTask4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTTask1 */
osThreadId_t UARTTask1Handle;
const osThreadAttr_t UARTTask1_attributes = {
  .name = "UARTTask1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorCtrlTasK1 */
osThreadId_t MotorCtrlTasK1Handle;
const osThreadAttr_t MotorCtrlTasK1_attributes = {
  .name = "MotorCtrlTasK1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorCtrlTasK2 */
osThreadId_t MotorCtrlTasK2Handle;
const osThreadAttr_t MotorCtrlTasK2_attributes = {
  .name = "MotorCtrlTasK2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorCtrlTasK3 */
osThreadId_t MotorCtrlTasK3Handle;
const osThreadAttr_t MotorCtrlTasK3_attributes = {
  .name = "MotorCtrlTasK3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorCtrlTasK4 */
osThreadId_t MotorCtrlTasK4Handle;
const osThreadAttr_t MotorCtrlTasK4_attributes = {
  .name = "MotorCtrlTasK4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};
/* Definitions for UART1Mutex01 */
osMutexId_t UART1Mutex01Handle;
const osMutexAttr_t UART1Mutex01_attributes = {
  .name = "UART1Mutex01"
};
/* USER CODE BEGIN PV */
osMutexId_t UART1Handle;
osMessageQueueId_t MotorQueueHandle;

uint8_t Motor_1=0x00;
uint8_t Motor_2=0x01;
uint8_t Motor_3=0x02;
uint8_t Motor_4=0x03;

Motor_Status_t motor1_n20 = { 1,0,0,0,0,0 };
Motor_Status_t motor2_n20 = { 2,0,0,0,0,0 };
Motor_Status_t motor3_n20 = { 3,0,0,0,0,0 };
Motor_Status_t motor4_n20 = { 4,0,0,0,0,0 };

// 全局电机状态数组
Motor_Status_t all_motors[4] = {0};

// 用于电机PWM控制的变量
uint8_t motor_pwm_tim_map[4] = {0, 1, 0, 2}; // 内部电机0,2使用TIM5, 内部电机1使用TIM9, 内部电机3使用TIM12
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
void GPIO_Task(void *argument);
void UART_Task(void *argument);
void Imu_Task(void *argument);
void PCA9685_Task(void *argument);
void Jetson_Task(void *argument);
void MotorSpeedTask(void *argument);
void UART_Task1(void *argument);
void MotorCtrl_TasK(void *argument);

/* USER CODE BEGIN PFP */
// 设置指定电机的PWM参数
void SetMotorPWM(uint8_t motor_id, float rpm);

// 停止指定电机的PWM输出
void StopMotorPWM(uint8_t motor_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 设置指定电机的PWM参数
void SetMotorPWM(uint8_t motor_id, float rpm)
{
    if (motor_id >= 4) return;  // 无效的电机ID

    uint32_t pulse;
    uint32_t max_pulse;
    float adjusted_rpm = fabsf(rpm);

    // 根据不同定时器的ARR值调整公式系数，使最大RPM对应接近ARR的脉冲宽度
    // 所有定时器现在都使用相同的ARR值13999，以便使用统一的公式
    max_pulse = 13999;  // 所有定时器的ARR值

    // 使用新的公式：当RPM达到最大值时，脉冲宽度接近ARR值
    // 假设最大RPM为380，则系数为 13999 / 380 = 36.84
    pulse = (uint32_t)(adjusted_rpm * 36.84f);

    // 限制脉冲宽度不超过ARR值
    if (pulse > max_pulse) pulse = max_pulse;

    switch(motor_id)
    {
        case 0:  // 内部电机0 (物理电机1) - 使用TIM5的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

            if (rpm >= 0) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
            } else {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
            break;

        case 1:  // 内部电机1 (物理电机2) - 使用TIM9的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);

            if (rpm >= 0) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            } else {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
            break;

        case 2:  // 内部电机2 (物理电机3) - 使用TIM5的通道3(正转)和通道4(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);

            if (rpm >= 0) {
                // 正转：通道3输出PWM（占空比与速度成正比），通道4保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pulse);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
            } else {
                // 反转：通道3保持低电平（占空比为0%），通道4输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pulse);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
            break;

        case 3:  // 内部电机3 (物理电机4) - 使用TIM12的通道1(正转)和通道2(反转)
            // 停止当前所有通道输出
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

            if (rpm >= 0) {
                // 正转：通道1输出PWM（占空比与速度成正比），通道2保持低电平（占空比为0%）
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
            } else {
                // 反转：通道1保持低电平（占空比为0%），通道2输出PWM（占空比与速度成正比）
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse);
            }

            // 启动PWM输出
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            break;
    }
}

// 停止指定电机的PWM输出
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_UART4_Init();

  // 系统启动信息输出
  char start_msg[] = "System Started Successfully!\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)start_msg, strlen(start_msg), 0xFFFF);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of UART1Mutex01 */
  UART1Mutex01Handle = osMutexNew(&UART1Mutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	UART1Handle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gpiotask */
  gpiotaskHandle = osThreadNew(GPIO_Task, NULL, &gpiotask_attributes);

  /* creation of uart_task */
  uart_taskHandle = osThreadNew(UART_Task, NULL, &uart_task_attributes);

  /* creation of imu_task */
  imu_taskHandle = osThreadNew(Imu_Task, NULL, &imu_task_attributes);

  /* creation of pca9685task */
  pca9685taskHandle = osThreadNew(PCA9685_Task, NULL, &pca9685task_attributes);

  /* creation of jetson_task */
  jetson_taskHandle = osThreadNew(Jetson_Task, NULL, &jetson_task_attributes);

  /* creation of MotorTask1 */
  MotorTask1Handle = osThreadNew(MotorSpeedTask, (void*) &motor1_n20, &MotorTask1_attributes);

  /* creation of MotorTask2 */
  MotorTask2Handle = osThreadNew(MotorSpeedTask, (void*) &motor2_n20, &MotorTask2_attributes);

  /* creation of MotorTask3 */
  MotorTask3Handle = osThreadNew(MotorSpeedTask, (void*) &motor3_n20, &MotorTask3_attributes);

  /* creation of MotorTask4 */
  MotorTask4Handle = osThreadNew(MotorSpeedTask, (void*) &motor4_n20, &MotorTask4_attributes);

  /* creation of UARTTask1 */
  UARTTask1Handle = osThreadNew(UART_Task1, NULL, &UARTTask1_attributes);

  /* creation of MotorCtrlTasK1 */
  MotorCtrlTasK1Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_1, &MotorCtrlTasK1_attributes);

  /* creation of MotorCtrlTasK2 */
  MotorCtrlTasK2Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_2, &MotorCtrlTasK2_attributes);

  /* creation of MotorCtrlTasK3 */
  MotorCtrlTasK3Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_3, &MotorCtrlTasK3_attributes);

  /* creation of MotorCtrlTasK4 */
  MotorCtrlTasK4Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_4, &MotorCtrlTasK4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 3;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 13999;  // 改为14000，对应约1.5kHz PWM频率 (84MHz/(3+1)/14000 = 1.5kHz)
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 7;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 13999;  // 改为14000，对应约1.5kHz PWM频率 (168MHz/(7+1)/14000 = 1.5kHz)
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 3;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 13999;  // 改为14000，对应约1.5kHz PWM频率 (84MHz/(3+1)/14000 = 1.5kHz)
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  // 使能UART5中断，调整优先级以避免与系统中断冲突
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DebugIO_GPIO_Port, DebugIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DebugIO_Pin */
  GPIO_InitStruct.Pin = DebugIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DebugIO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_GPIO_Task */
/**
  * @brief  Function implementing the gpiotask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GPIO_Task */
void GPIO_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the uart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task */
void UART_Task(void *argument)
{
  /* USER CODE BEGIN UART_Task */
  uint32_t cnt = 0;
  char uart_send_buf[50];

  // 等待系统稳定
  osDelay(1000);

  while(1)
  {
      cnt++;
//      sprintf(uart_send_buf, "UART Message Count: %lu\r\n", cnt);

//      // 发送数据到串口
//      HAL_UART_Transmit(&huart4, (uint8_t*)uart_send_buf, strlen(uart_send_buf), 0xFFFF);

      // 每隔1秒发送一次
      osDelay(1000);
  }
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_Imu_Task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task */
void Imu_Task(void *argument)
{
  /* USER CODE BEGIN Imu_Task */
  MPU6050_Data_t imu_data;
  char imu_send_buf[100];
  HAL_StatusTypeDef status;

  // 等待系统稳定
  osDelay(1000);

  // 初始化MPU6050传感器
  status = BSP_MPU6050_Init(&hi2c1);
  if(status != HAL_OK)
  {
      // 初始化失败，发送错误信息
      sprintf(imu_send_buf, "MPU6050 Initialization Failed!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }
  else
  {
      // 初始化成功，发送确认信息
      sprintf(imu_send_buf, "MPU6050 Initialization Success!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }

  /* Infinite loop */
  for(;;)
  {
    // 读取MPU6050传感器数据
    status = BSP_MPU6050_Read_Data(&hi2c1, &imu_data);

    if(status == HAL_OK)
    {
        // 格式化并发送IMU数据
        sprintf(imu_send_buf,
                "IMU: Accel[X:%.2fg,Y:%.2fg,Z:%.2fg] Gyro[X:%.2fdps,Y:%.2fdps,Z:%.2fdps] Temp:%.2fc\r\n",
                imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                imu_data.temperature_c);

        // 更新IMU数据到综合姿态信息
        extern void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                                  float gyro_x, float gyro_y, float gyro_z,
                                  float temperature);
        UpdateIMUData(imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                      imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                      imu_data.temperature_c);
    }
    else
    {
        sprintf(imu_send_buf, "MPU6050 Read Error!\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
    }

    // 每隔100ms读取一次数据 (10Hz)
    osDelay(100);
  }
  /* USER CODE END Imu_Task */
}

/* USER CODE BEGIN Header_PCA9685_Task */
/**
* @brief Function implementing the pca9685task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PCA9685_Task */
void PCA9685_Task(void *argument)
{
  /* USER CODE BEGIN PCA9685_Task */
  char servo_send_buf[100];
  HAL_StatusTypeDef status;

  // 等待系统稳定
  osDelay(1000);

  // 初始化PCA9685，设置PWM频率为50Hz（舵机标准频率）
  status = HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR, 5, 100);
  if(status == HAL_OK)
  {
      PCA9685_Init(&hi2c2, 50.0f);  // 50Hz for servo control

      // 发送初始化成功信息
      sprintf(servo_send_buf, "PCA9685 Initialization Success!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
  }
  else
  {
      // 发送初始化失败信息
      sprintf(servo_send_buf, "PCA9685 Device Not Found!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
  }

  /* Infinite loop */
  for(;;)
  {
      // 现在舵机控制由UART接收消息驱动，这里只负责检查设备状态
      if(status != HAL_OK)
      {
          // 如果设备未找到，尝试重新初始化
          status = HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR, 5, 100);
          if(status == HAL_OK)
          {
              PCA9685_Init(&hi2c2, 50.0f);  // 50Hz for servo control

              // 发送重新初始化成功信息
              sprintf(servo_send_buf, "PCA9685 Re-initialization Success!\r\n");
              HAL_UART_Transmit(&huart4, (uint8_t*)servo_send_buf, strlen(servo_send_buf), 0xFFFF);
          }
      }

      // 延迟较长时间，因为主要控制逻辑在UART接收部分
      osDelay(1000);
  }
  /* USER CODE END PCA9685_Task */
}

/* USER CODE BEGIN Header_Jetson_Task */
/**
* @brief Function implementing the jetson_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Jetson_Task */
void Jetson_Task(void *argument)
{
  /* USER CODE BEGIN Jetson_Task */
  char debug_msg[100];

  // 用于100Hz姿态回传的时间跟踪
  uint32_t last_pose_transmit = 0;
  const uint32_t POSE_TRANSMIT_INTERVAL = 10; // 10ms = 100Hz

  // 初始化UART通信
  JetsonUart_Init();

  // 等待UART完全初始化
  osDelay(100);

  // 启动UART5中断接收
  if(JetsonUart_StartReceiveIT() != HAL_OK) {
      sprintf(debug_msg, "Failed to start UART5 IT receive\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  } else {
      sprintf(debug_msg, "UART5 IT receive started successfully\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  }

  // 等待系统稳定
  osDelay(1000);

  // 发送初始化完成消息
  sprintf(debug_msg, "Jetson Comm Task Started at 100Hz\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);

  /* Infinite loop */
  for(;;)
  {
    // 100Hz通信速率，即每10ms执行一次循环
    // 数据接收由中断处理，这里可以做其他事情

    // 更新所有电机数据到综合姿态信息
    extern void UpdateMotorData(int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4,
                               float rpm1, float rpm2, float rpm3, float rpm4);
    UpdateMotorData(all_motors[0].direction, all_motors[1].direction,
                   all_motors[2].direction, all_motors[3].direction,
                   all_motors[0].speed_rpm, all_motors[1].speed_rpm,
                   all_motors[2].speed_rpm, all_motors[3].speed_rpm);

    osDelay(10);

    // 检查接收超时并处理
    Jetson_CheckReceiveTimeout();
    // 定期清理可能的陈旧数据
    Jetson_ClearStaleData();

    // 检查姿态是否发生变化，如果变化则发送
    if(CheckPoseChanged()) {
        // 发送当前姿态
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
    // 同时保持最小发送间隔，避免过于频繁发送
    else if((HAL_GetTick() - last_pose_transmit) >= POSE_TRANSMIT_INTERVAL) {
        // 即使没有变化，也定期发送以维持通信
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
  }
  /* USER CODE END Jetson_Task */
}

/* USER CODE BEGIN Header_MotorSpeedTask */
/**
* @brief Function implementing the MotorTask1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorSpeedTask */
void MotorSpeedTask(void *argument)
{
  /* USER CODE BEGIN MotorSpeedTask */
  /* Infinite loop */

			 // 定义TIM句柄数组
	  	TIM_HandleTypeDef* tim_handles[] = { NULL,&htim1, &htim2,&htim3,&htim8};//NULL只是用来占位的
		// 定义结构体实例,互斥访问时需要放到函数里
			Motor_Status_t motor_n20 = { ((Motor_Status_t*)argument)->dev,0,0,0,0 };
			uint8_t Motor_id=motor_n20.dev;//(1~4)
			TIM_HandleTypeDef* tim_handles_self=tim_handles[Motor_id];

    // 初始化上一次计数器值为当前值，防止第一次计算出错
    motor_n20.last_counter = __HAL_TIM_GET_COUNTER(tim_handles_self);

    // 获取当前 Tick 时间，用于精确延时(即作为vTaskDelayUntil的延时起点,记录的是上一个周期理论唤醒时间,这个值会被vTaskDelayUntil更新),这个时间是从SystemTick里面读的
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(SPEED_SAMPLE_TIME_MS);  //将周期的单位转换为tick

    while (1)
    {
 
            // ================= A. 数据采集 =================
            // 根据Motor_id选择对应的TIM
            uint16_t current_cnt = __HAL_TIM_GET_COUNTER(tim_handles[Motor_id]);
        
            // ================= B. 核心解算 =================
            
            // 1. 计算增量 (Delta)
            int16_t delta = (int16_t)(current_cnt - motor_n20.last_counter);
            
            // 更新状态结构体
            motor_n20.encode_delta = delta;
            motor_n20.last_counter = current_cnt;
            motor_n20.total_count += delta; // 如果需要记录跑了多远
            
            // 2. 计算 RPM (转速)
            float rpm = ((float)delta) * (60000.0f / SPEED_SAMPLE_TIME_MS) / PULSES_PER_REV_OUTPUT;
            motor_n20.speed_rpm = rpm;
            
            // 3. 判断方向
            if (delta > 0) {
                motor_n20.direction = 1;
            }
            else if (delta < 0) {
                motor_n20.direction = -1;
            }
            else {
                motor_n20.direction = 0;
            }
            
            /* 将结构体写入队列 */
            osMessageQueuePut(MotorQueueHandle, &motor_n20, NULL, osWaitForever);
            
            // 更新电机数据到综合姿态信息
            extern void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm);
            // 根据当前任务的设备ID来确定更新哪个电机的数据
            UpdateSingleMotorData(motor_n20.dev - 1, motor_n20.direction, motor_n20.speed_rpm);
            
            // 同时更新全局电机状态数组
            all_motors[motor_n20.dev - 1] = motor_n20;
            
            // ================= C. 任务调度 =================
            // 使用 vTaskDelayUntil 保证采样的绝对周期性
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        

  /* USER CODE END MotorSpeedTask */
}

/* USER CODE BEGIN Header_UART_Task1 */
/**
* @brief Function implementing the UARTTask1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task1 */
void UART_Task1(void *argument)
{
  /* USER CODE BEGIN UART_Task1 */
  /* Infinite loop */
    Motor_Status_t MotorDataStructure_tx;
    (void)argument; // 避免未使用参数警告，根据CMSIS标准

    while(1) {
        // 接收 - 注意：portMAX_DELAY 是 FreeRTOS 宏，CMSIS 应使用 osWaitForever
        if(osMessageQueueGet(MotorQueueHandle, &MotorDataStructure_tx, NULL, osWaitForever) == osOK) {

            // 方法1: 发送RPM的原始浮点值（4字节）
            union {
                float rpm_float;
                uint8_t rpm_bytes[4];
            } rpm_converter;

            rpm_converter.rpm_float = MotorDataStructure_tx.speed_rpm;

            // 添加安全检查，确保只发送有效的数据
            if(osMutexAcquire(UART1Handle, osWaitForever) == osOK) {
                // 检查是否包含可能的无效值（如0xA5），如果四个字节都是0xA5则跳过发送
                uint8_t invalid_pattern[] = {0xA5, 0xA5, 0xA5, 0xA5};
                if(memcmp(rpm_converter.rpm_bytes, invalid_pattern, 4) != 0) {
                    // 发送4字节的浮点数RPM值
                    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, rpm_converter.rpm_bytes, 4, HAL_MAX_DELAY);

                    // 检查传输状态
                    if(status != HAL_OK) {
                        // 可以在这里添加错误处理
                        // 例如：通过调试串口输出错误信息
                        char errorMsg[] = "UART1 transmit error\r\n";
                        // HAL_UART_Transmit(&huart4, (uint8_t*)errorMsg, strlen(errorMsg), 0xFFFF);
                    }
                }

                // 也可以同时发送编码器增量值
                // HAL_UART_Transmit(&huart1, (uint8_t*)&MotorDataStructure_tx.encode_delta, 2, HAL_MAX_DELAY);

                osMutexRelease(UART1Handle);
            }

            // 方法2: 如果只需要发送整数部分，可以保留原来的实现方式
            // uint16_t rpm_int = (uint16_t)(MotorDataStructure_tx.speed_rpm * 100); // 乘以100保留两位小数
            // uint8_t speed_rpm_H = (rpm_int >> 8) & 0xFF;
            // uint8_t speed_rpm_L = rpm_int & 0xFF;
        }
    }
  /* USER CODE END UART_Task1 */
}

/* USER CODE BEGIN Header_MotorCtrl_TasK */
/**
* @brief Function implementing the MotorCtrlTasK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCtrl_TasK */
void MotorCtrl_TasK(void *argument)
{
  /* USER CODE BEGIN MotorCtrl_TasK */
  /* Infinite loop */
	uint8_t motor_id=*(uint8_t*)argument;
  int8_t direction;
  float speed_rpm;

  for(;;)
  {
    // 获取上位机发来的电机命令
    uint8_t motor_cmd = GetMotorCommandType(motor_id);  // 获取对应编号电机的命令类型
    GetMotorCommand(motor_id, &direction, &speed_rpm);  // 获取方向和速度

    switch(motor_cmd) {
      case 0:  // 慢刹车 - 两个端口保持低电平
        StopMotorPWM(motor_id);
        // 设置GPIO状态为低电平
        switch(motor_id) {
          case 0:  //内部电机0 (物理电机1)
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_1_GPIO_Port, MotorDirectionControl_IO1_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_2_GPIO_Port, MotorDirectionControl_IO1_2_Pin, GPIO_PIN_RESET);
            break;
          case 1:  //内部电机1 (物理电机2)
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_1_GPIO_Port, MotorDirectionControl_IO2_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_2_GPIO_Port, MotorDirectionControl_IO2_2_Pin, GPIO_PIN_RESET);
            break;
          case 2:  //内部电机2 (物理电机3)
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_1_GPIO_Port, MotorDirectionControl_IO3_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_2_GPIO_Port, MotorDirectionControl_IO3_2_Pin, GPIO_PIN_RESET);
            break;
          case 3:  //内部电机3 (物理电机4)
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_1_GPIO_Port, MotorDirectionControl_IO4_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_2_GPIO_Port, MotorDirectionControl_IO4_2_Pin, GPIO_PIN_RESET);
            break;
        }
        break;

      case 1:  // 快刹车 - 两个端口保持高电平
        StopMotorPWM(motor_id);
        // 设置GPIO状态为高电平
        switch(motor_id) {
          case 0:  //内部电机0 (物理电机1)
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_1_GPIO_Port, MotorDirectionControl_IO1_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_2_GPIO_Port, MotorDirectionControl_IO1_2_Pin, GPIO_PIN_SET);
            break;
          case 1:  //内部电机1 (物理电机2)
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_1_GPIO_Port, MotorDirectionControl_IO2_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_2_GPIO_Port, MotorDirectionControl_IO2_2_Pin, GPIO_PIN_SET);
            break;
          case 2:  //内部电机2 (物理电机3)
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_1_GPIO_Port, MotorDirectionControl_IO3_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_2_GPIO_Port, MotorDirectionControl_IO3_2_Pin, GPIO_PIN_SET);
            break;
          case 3:  //内部电机3 (物理电机4)
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_1_GPIO_Port, MotorDirectionControl_IO4_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_2_GPIO_Port, MotorDirectionControl_IO4_2_Pin, GPIO_PIN_SET);
            break;
        }
        break;

      case 2:  // 反转 - 速度为负值
        // 电机x 反转 速度-a 的命令对应x的第2个端口输出对应pulse的PWM波,第1个端口保持低电平
        SetMotorPWM(motor_id, -speed_rpm);  // 使用负速度值表示反转
        break;

      case 3:  // 正转 - 速度为正值
        // 电机x 正转 速度a 的命令对应x的第1个端口输出对应pulse的PWM波,第2个端口保持低电平
        SetMotorPWM(motor_id, speed_rpm);  // 使用正速度值表示正转
        break;

      default:  // 默认情况，停止电机
        StopMotorPWM(motor_id);
        // 设置GPIO状态为低电平
        switch(motor_id) {
          case 0:  //内部电机0 (物理电机1)
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_1_GPIO_Port, MotorDirectionControl_IO1_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO1_2_GPIO_Port, MotorDirectionControl_IO1_2_Pin, GPIO_PIN_RESET);
            break;
          case 1:  //内部电机1 (物理电机2)
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_1_GPIO_Port, MotorDirectionControl_IO2_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO2_2_GPIO_Port, MotorDirectionControl_IO2_2_Pin, GPIO_PIN_RESET);
            break;
          case 2:  //内部电机2 (物理电机3)
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_1_GPIO_Port, MotorDirectionControl_IO3_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO3_2_GPIO_Port, MotorDirectionControl_IO3_2_Pin, GPIO_PIN_RESET);
            break;
          case 3:  //内部电机3 (物理电机4)
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_1_GPIO_Port, MotorDirectionControl_IO4_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MotorDirectionControl_IO4_2_GPIO_Port, MotorDirectionControl_IO4_2_Pin, GPIO_PIN_RESET);
            break;
        }
        break;
    }

    osDelay(10);  // 10ms检查一次命令状态
  }
  /* USER CODE END MotorCtrl_TasK */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  char error_msg[] = "Error_Handler: System Error Occurred!\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0xFFFF);

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
