#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief  速度计算任务,使用固定周期进行计算,以防止速度横跳的问题
 * @note   在 FreeRTOS 中创建一个任务调用此函数
 */
 
 
// 
//extern 	QueueHandle_t MotorQueueHandle;
//extern  TIM_HandleTypeDef htim3;
//osMutexId_t UART1Handle;
//osMessageQueueId_t MotorQueueHandle;

//    1 // 接收端代码示例
//    2 uint8_t received_bytes[4];
//    3 // ... 从串口接收4个字节到received_bytes数组 ...
//    4
//    5 union {
//    6     float rpm_float;
//    7     uint8_t rpm_bytes[4];
//    8 } received_converter;
//    9
//   10 // 将接收到的字节复制到联合体
//   11 for(int i = 0; i < 4; i++) {
//   12     received_converter.rpm_bytes[i] = received_bytes[i];
//   13 }
//   14
//   15 // 现在可以使用浮点数了
//   16 float received_rpm = received_converter.rpm_float;