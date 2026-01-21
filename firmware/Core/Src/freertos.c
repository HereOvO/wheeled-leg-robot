/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  
#include "queue.h"  
#include "main.h"
#include "cmsis_os.h"
#include "jetson_uart.h"
#include "math.h"

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
/* USER CODE BEGIN Variables */
extern Motor_Status_t motor0_n20;
extern Motor_Status_t motor1_n20;
extern Motor_Status_t motor2_n20;
extern Motor_Status_t motor3_n20;
extern uint8_t Motor_0;
extern uint8_t Motor_1;
extern uint8_t Motor_2;
extern uint8_t Motor_3;

//��Բ�ͬ�Ķ�ʱ����������޷�
//uint16_t ulMaxPulse_TIM5_12=MAX_MOTOR_PWM_PULSE_5_12;
//uint16_t ulMaxPulse_TIM9=MAX_MOTOR_PWM_PULSE_9;

/* USER CODE END Variables */
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
/* Definitions for Motor_PID_Task */
osThreadId_t Motor_PID_TaskHandle;
const osThreadAttr_t Motor_PID_Task_attributes = {
  .name = "Motor_PID_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART1Mutex01 */
osMutexId_t UART1Mutex01Handle;
const osMutexAttr_t UART1Mutex01_attributes = {
  .name = "UART1Mutex01"
};
/* Definitions for MotorPIDOutPulseMutex */
osMutexId_t MotorPIDOutPulseMutexHandle;
const osMutexAttr_t MotorPIDOutPulseMutex_attributes = {
  .name = "MotorPIDOutPulseMutex"
};
/* Definitions for MotorStatusMutex */
osMutexId_t MotorStatusMutexHandle;
const osMutexAttr_t MotorStatusMutex_attributes = {
  .name = "MotorStatusMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void GPIO_Task(void *argument);
extern void UART_Task(void *argument);
extern void Imu_Task(void *argument);
extern void PCA9685_Task(void *argument);
extern void Jetson_Task(void *argument);
extern void MotorSpeedTask(void *argument);
extern void MotorCtrl_TasK(void *argument);
extern void Motor_PID_TaskFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of UART1Mutex01 */
  UART1Mutex01Handle = osMutexNew(&UART1Mutex01_attributes);

  /* creation of MotorPIDOutPulseMutex */
  MotorPIDOutPulseMutexHandle = osMutexNew(&MotorPIDOutPulseMutex_attributes);

  /* creation of MotorStatusMutex */
  MotorStatusMutexHandle = osMutexNew(&MotorStatusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

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
  MotorTask1Handle = osThreadNew(MotorSpeedTask, (void*) &motor0_n20, &MotorTask1_attributes);

  /* creation of MotorTask2 */
  MotorTask2Handle = osThreadNew(MotorSpeedTask, (void*) &motor1_n20, &MotorTask2_attributes);

  /* creation of MotorTask3 */
  MotorTask3Handle = osThreadNew(MotorSpeedTask, (void*) &motor2_n20, &MotorTask3_attributes);

  /* creation of MotorTask4 */
  MotorTask4Handle = osThreadNew(MotorSpeedTask, (void*) &motor3_n20, &MotorTask4_attributes);

  /* creation of MotorCtrlTasK1 */
  MotorCtrlTasK1Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_0, &MotorCtrlTasK1_attributes);

  /* creation of MotorCtrlTasK2 */
  MotorCtrlTasK2Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_1, &MotorCtrlTasK2_attributes);

  /* creation of MotorCtrlTasK3 */
  MotorCtrlTasK3Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_2, &MotorCtrlTasK3_attributes);

  /* creation of MotorCtrlTasK4 */
  MotorCtrlTasK4Handle = osThreadNew(MotorCtrl_TasK, (void*) &Motor_3, &MotorCtrlTasK4_attributes);

  /* creation of Motor_PID_Task */
  Motor_PID_TaskHandle = osThreadNew(Motor_PID_TaskFunc, NULL, &Motor_PID_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_GPIO_Task */
/**
  * @brief  Function implementing the gpiotask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_GPIO_Task */
void GPIO_Task(void *argument)
{
  /* USER CODE BEGIN GPIO_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPIO_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

