//=================include===========================
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "uart.h"

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
//=================self PV===========================
osMutexId_t UART4mutex_id;
//=====================其它函数==================================


//=====================任务函数==================================
void UART_Task(void *argument)
{
  /* USER CODE BEGIN UART_Task */
  unsigned long cnt = 0;
  char uart_send_buf[50];

  // 等待系统稳定
  osDelay(1000);

  while(1)
  {
      cnt++;
      sprintf(uart_send_buf, "UART Message Count: %lu\r\n", cnt);

      // 发送数据到串口
      HAL_UART_Transmit(&huart4, (uint8_t*)uart_send_buf, strlen(uart_send_buf), 0xFFFF);

      // 每隔1秒发送一次
      osDelay(1000);
  }
  /* USER CODE END UART_Task */
}