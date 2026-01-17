/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.c
  * @brief          : UART communication implementation for Jetson Nano
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
#include "jetson_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"  // 需要访问huart4和huart5
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"  // 添加FreeRTOS头文件以使用互斥锁
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief 舵机控制消息结构体
 */
typedef struct {
    uint8_t servo_id;      /*!< 舵机ID (0-15) */
    uint16_t angle;        /*!< 目标角度 (0-180度) */
    uint16_t duration;     /*!< 执行时间 (ms) */
} ServoControlMsg_t;

/**
 * @brief 多舵机姿态结构体
 */
typedef struct {
    uint16_t angles[16];   /*!< 16个舵机的角度值 */
    uint32_t timestamp;    /*!< 时间戳 */
} MultiServoPose_t;

/**
 * @brief IMU数据结构体
 */
typedef struct {
    float accel_x_g;       /*!< X轴加速度 (g) */
    float accel_y_g;       /*!< Y轴加速度 (g) */
    float accel_z_g;       /*!< Z轴加速度 (g) */
    float gyro_x_dps;      /*!< X轴角速度 (°/s) */
    float gyro_y_dps;      /*!< Y轴角速度 (°/s) */
    float gyro_z_dps;      /*!< Z轴角速度 (°/s) */
    float temperature_c;   /*!< 温度 (°C) */
} IMUData_t;

/**
 * @brief 电机数据结构体
 */
typedef struct {
    int8_t direction[4];          /*!< 4个电机的转向 (1:正转, -1:反转, 0:停止) */
    float speed_rpm[4];           /*!< 4个电机的转速 (RPM) */
    uint32_t timestamp;           /*!< 时间戳 */
} MotorData_t;

/**
 * @brief 综合姿态信息结构体（舵机+IMU+电机）
 */
typedef struct {
    MultiServoPose_t servo_pose;  /*!< 舵机姿态 */
    IMUData_t imu_data;           /*!< IMU数据 */
    MotorData_t motor_data;       /*!< 电机数据 */
    uint32_t timestamp;           /*!< 综合时间戳 */
} ComprehensivePose_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static uint8_t uart5_rx_buffer[1024];  /*!< UART5接收缓冲区，增大缓冲区以应对高频数据 */
static uint8_t uart5_rx_index = 0;      /*!< UART5接收索引 */
static uint8_t uart5_frame_complete = 0; /*!< 帧完成标志 */
static uint8_t uart5_rx_data;           /*!< UART5接收数据 */

static JetsonMessage_t received_msg;     /*!< 用于接收消息的临时缓冲区 */

static uint32_t last_receive_tick = 0;  /*!< 最后接收时间戳 */
static const uint32_t RECEIVE_TIMEOUT_MS = 200;  /*!< 接收超时时间 (200ms) */

static MultiServoPose_t current_pose = {0};     /*!< 当前舵机姿态 */
static uint32_t last_pose_update = 0;           /*!< 上次姿态更新时间 */

static ComprehensivePose_t comprehensive_pose = {0};  /*!< 综合姿态信息（舵机+IMU+电机） */

// 存储上位机发来的电机控制命令
static int8_t motor_cmd_direction[4] = {0};  /*!< 上位机发来的电机转向命令 */
static float motor_cmd_speed[4] = {0.0f};   /*!< 上位机发来的电机转速命令 */
static uint8_t motor_cmd_type[4] = {0};     /*!< 上位机发来的电机命令类型 (0=慢刹车, 1=快刹车, 2=反转, 3=正转) */

// 电机命令数组的互斥锁，用于保护中断和任务之间的数据访问
static osMutexId_t motor_cmd_mutex = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

// 解析舵机控制消息
HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint8_t length);

// 发送当前姿态消息
HAL_StatusTypeDef SendCurrentPose(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 串口接收单字节缓冲区
static uint8_t uart_rx_byte;

// 启动UART5中断接收
HAL_StatusTypeDef JetsonUart_StartReceiveIT(void)
{
    // 确保huart5句柄有效
    if(&huart5 == NULL) {
        return HAL_ERROR;
    }

    // 启动单字节中断接收
    return HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
}

// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5) {
        // 更新最后接收时间戳
        last_receive_tick = HAL_GetTick();

        // 处理接收到的数据
        Jetson_ProcessReceivedData(&uart_rx_byte, 1);

        // 重新启动接收下一个字节
        if(HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1) != HAL_OK) {
            // 如果重新启动接收失败，可以发送错误信息
            char error_msg[] = "UART5 Re-start receive error\r\n";
            HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0xFFFF);
        }
    }
}

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/* USER CODE BEGIN 1 */
// UART错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5) {
        // 检查是否真的是错误（错误码非0）
        if(huart->ErrorCode != HAL_UART_ERROR_NONE) {
            // 清除错误状态
            huart->ErrorCode = HAL_UART_ERROR_NONE;
        }
        // 无论是否有实际错误，都重新启动接收
        HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    }
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
// 检查接收超时的函数
void Jetson_CheckReceiveTimeout(void)
{
    // 检查是否超过接收超时时间
    if(uart5_rx_index > 0 && (HAL_GetTick() - last_receive_tick) > RECEIVE_TIMEOUT_MS) {
        // 超时，重置接收索引，丢弃未完成的消息
        uart5_rx_index = 0;
    }
}


/**
 * @brief 解析舵机控制消息
 * @param buffer 消息缓冲区
 * @param length 消息长度
 * @return HAL_StatusTypeDef HAL_OK 成功，HAL_ERROR 失败
 *
 * 消息格式: "SERVO,<servo_id>,<angle>,<duration>\n"
 * 例如: "SERVO,0,90,1000\n" - 控制0号舵机转到90度，用时1000ms
 */
HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint8_t length)
{
    if(buffer == NULL || length == 0) {
        return HAL_ERROR;
    }

    // 简单解析消息（实际应用中应使用更健壮的解析方法）
    char msg_str[64];
    if(length >= sizeof(msg_str)) {
        return HAL_ERROR; // 消息太长
    }

    // 复制到临时字符串
    for(int i = 0; i < length && i < sizeof(msg_str)-1; i++) {
        msg_str[i] = (char)buffer[i];
    }
    msg_str[length] = '\0';

    // 检查消息头
    if(strncmp(msg_str, "SERVO,", 6) != 0) {
        return HAL_ERROR; // 不是舵机控制消息
    }

    // 解析参数
    char *token = strtok(msg_str, ",");
    if(token == NULL || strcmp(token, "SERVO") != 0) {
        return HAL_ERROR;
    }

    // 解析舵机ID
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint8_t servo_id = atoi(token);

    // 解析角度
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint16_t angle = atoi(token);

    // 解析持续时间
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint16_t duration = atoi(token);

    // 限制范围
    if(servo_id >= 16 || angle > 180) {
        return HAL_ERROR;
    }

    // 更新当前姿态
    current_pose.angles[servo_id] = angle;
    current_pose.timestamp = HAL_GetTick();
    last_pose_update = HAL_GetTick();

    // 同时更新综合姿态信息
    comprehensive_pose.servo_pose.angles[servo_id] = angle;
    comprehensive_pose.servo_pose.timestamp = HAL_GetTick();

    // 调用PCA9685控制函数（需要外部提供）
    extern void PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t angle);
    extern I2C_HandleTypeDef hi2c2;  /*!< 外部I2C句柄 */

    // 直接设置舵机角度，不使用缓慢移动
    PCA9685_SetAngle(&hi2c2, servo_id, angle);

    return HAL_OK;
}

/**
 * @brief 解析电机控制消息
 * @param buffer 消息缓冲区
 * @param length 消息长度
 * @return HAL_StatusTypeDef HAL_OK 成功，HAL_ERROR 失败
 *
 * 消息格式: "MOTOR,<motor_id>,<command>,<speed_rpm>\n"
 * command: 0=慢刹车, 1=快刹车, 2=反转, 3=正转
 * 例如: "MOTOR,0,3,50.5\n" - 控制0号电机正转，转速50.5 RPM
 */
HAL_StatusTypeDef ParseMotorControlMessage(uint8_t *buffer, uint8_t length)
{
    if(buffer == NULL || length == 0) {
        return HAL_ERROR;
    }

    // 简单解析消息（实际应用中应使用更健壮的解析方法）
    char msg_str[128];
    if(length >= sizeof(msg_str)) {
        return HAL_ERROR; // 消息太长
    }

    // 复制到临时字符串
    for(int i = 0; i < length && i < sizeof(msg_str)-1; i++) {
        msg_str[i] = (char)buffer[i];
    }
    msg_str[length] = '\0';

    // 检查消息头
    if(strncmp(msg_str, "MOTOR,", 6) != 0) {
        return HAL_ERROR; // 不是电机控制消息
    }

    // 解析参数
    char *token = strtok(msg_str, ",");
    if(token == NULL || strcmp(token, "MOTOR") != 0) {
        return HAL_ERROR;
    }

    // 解析电机ID
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint8_t motor_id = atoi(token);

    // 解析命令
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint8_t command = (uint8_t)atoi(token);

    // 解析转速
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    float speed_rpm = atof(token);

    // 限制范围
    if(motor_id >= 4 || command > 3) {
        return HAL_ERROR;
    }

    // 将命令转换为内部表示
    int8_t direction = 0;  // 默认停止
    switch(command) {
        case 0:  // 慢刹车
            direction = 0;  // 停止，但可能需要特殊的慢刹车逻辑
            speed_rpm = 0.0f;
            break;
        case 1:  // 快刹车
            direction = 0;  // 停止，但可能需要特殊的快刹车逻辑
            speed_rpm = 0.0f;
            break;
        case 2:  // 反转
            direction = -1;
            break;
        case 3:  // 正转
            direction = 1;
            break;
        default:
            return HAL_ERROR;
    }

    // 存储上位机发来的电机控制命令（使用原子操作保护）
    if(motor_id < 4) {
        // 在中断中禁用中断以保护数据一致性
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        motor_cmd_direction[motor_id] = direction;
        motor_cmd_speed[motor_id] = speed_rpm;
        motor_cmd_type[motor_id] = command;  // 存储原始命令类型

        // 恢复中断状态
        __set_PRIMASK(primask);
    }

    // 这里可以添加对电机的控制逻辑（目前只更新内部状态）
    // extern void ControlMotor(uint8_t motor_id, int8_t direction, float speed_rpm);
    // ControlMotor(motor_id, direction, speed_rpm);

    return HAL_OK;
}

/**
 * @brief 发送当前姿态消息
 * @return HAL_StatusTypeDef HAL_OK 成功，其他值 失败
 *
 * 消息格式: "POSE,<servo0>,<servo1>,...,<servo15>,<timestamp>,<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<temperature>,<dir1>,<dir2>,<dir3>,<dir4>,<rpm1>,<rpm2>,<rpm3>,<rpm4>\n"
 */
HAL_StatusTypeDef SendCurrentPose(void)
{
    char pose_msg[512]; // 增大缓冲区以容纳更多数据
    sprintf(pose_msg, "POSE,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n",
            comprehensive_pose.servo_pose.angles[0], comprehensive_pose.servo_pose.angles[1],
            comprehensive_pose.servo_pose.angles[2], comprehensive_pose.servo_pose.angles[3],
            comprehensive_pose.servo_pose.angles[4], comprehensive_pose.servo_pose.angles[5],
            comprehensive_pose.servo_pose.angles[6], comprehensive_pose.servo_pose.angles[7],
            comprehensive_pose.servo_pose.angles[8], comprehensive_pose.servo_pose.angles[9],
            comprehensive_pose.servo_pose.angles[10], comprehensive_pose.servo_pose.angles[11],
            comprehensive_pose.servo_pose.angles[12], comprehensive_pose.servo_pose.angles[13],
            comprehensive_pose.servo_pose.angles[14], comprehensive_pose.servo_pose.angles[15],
            comprehensive_pose.servo_pose.timestamp,
            comprehensive_pose.imu_data.accel_x_g, comprehensive_pose.imu_data.accel_y_g, comprehensive_pose.imu_data.accel_z_g,
            comprehensive_pose.imu_data.gyro_x_dps, comprehensive_pose.imu_data.gyro_y_dps, comprehensive_pose.imu_data.gyro_z_dps,
            comprehensive_pose.imu_data.temperature_c,
            comprehensive_pose.motor_data.direction[0], comprehensive_pose.motor_data.direction[1],
            comprehensive_pose.motor_data.direction[2], comprehensive_pose.motor_data.direction[3],
            comprehensive_pose.motor_data.speed_rpm[0], comprehensive_pose.motor_data.speed_rpm[1],
            comprehensive_pose.motor_data.speed_rpm[2], comprehensive_pose.motor_data.speed_rpm[3]);

    return HAL_UART_Transmit(&huart5, (uint8_t*)pose_msg, strlen(pose_msg), 0xFFFF);
}

/**
 * @brief 检查姿态是否发生变化
 * @return uint8_t 1-发生变化，0-未发生变化
 *
 * 检查舵机角度是否发生变化，可根据需要扩展检查IMU数据变化
 */
uint8_t CheckPoseChanged(void)
{
    static ComprehensivePose_t last_pose = {0};
    uint8_t changed = 0;

    // 检查舵机角度是否发生变化
    for(int i = 0; i < 16; i++) {
        if(comprehensive_pose.servo_pose.angles[i] != last_pose.servo_pose.angles[i]) {
            changed = 1;
            break;
        }
    }

    // 如果舵机角度没变，检查IMU数据是否发生变化（可选）
    if(!changed) {
        // 可以根据需要决定是否检查IMU数据变化
        // 这里暂时只检查舵机角度变化
    }

    if(changed) {
        // 更新last_pose
        for(int i = 0; i < 16; i++) {
            last_pose.servo_pose.angles[i] = comprehensive_pose.servo_pose.angles[i];
        }
        last_pose.servo_pose.timestamp = comprehensive_pose.servo_pose.timestamp;
    }

    return changed;
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

// 定期清理接收缓冲区的函数，防止长时间累积数据
void Jetson_ClearStaleData(void)
{
    // 如果接收索引长时间未被重置且数值较大，则强制清空
    if(uart5_rx_index > 50) {  // 调整阈值以平衡响应速度和稳定性
        // 重置接收索引
        uart5_rx_index = 0;
    }
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */

// 更新IMU数据
void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                   float gyro_x, float gyro_y, float gyro_z,
                   float temperature)
{
    comprehensive_pose.imu_data.accel_x_g = accel_x;
    comprehensive_pose.imu_data.accel_y_g = accel_y;
    comprehensive_pose.imu_data.accel_z_g = accel_z;
    comprehensive_pose.imu_data.gyro_x_dps = gyro_x;
    comprehensive_pose.imu_data.gyro_y_dps = gyro_y;
    comprehensive_pose.imu_data.gyro_z_dps = gyro_z;
    comprehensive_pose.imu_data.temperature_c = temperature;
    comprehensive_pose.timestamp = HAL_GetTick();
}

// 更新电机数据
void UpdateMotorData(int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4,
                     float rpm1, float rpm2, float rpm3, float rpm4)
{
    // 一次性更新所有电机数据
    comprehensive_pose.motor_data.direction[0] = dir1;
    comprehensive_pose.motor_data.direction[1] = dir2;
    comprehensive_pose.motor_data.direction[2] = dir3;
    comprehensive_pose.motor_data.direction[3] = dir4;

    comprehensive_pose.motor_data.speed_rpm[0] = rpm1;
    comprehensive_pose.motor_data.speed_rpm[1] = rpm2;
    comprehensive_pose.motor_data.speed_rpm[2] = rpm3;
    comprehensive_pose.motor_data.speed_rpm[3] = rpm4;

    comprehensive_pose.motor_data.timestamp = HAL_GetTick();
    comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳
}

// 更新单个电机数据
void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm)
{
    if(motor_id < 4) {
        comprehensive_pose.motor_data.direction[motor_id] = direction;
        comprehensive_pose.motor_data.speed_rpm[motor_id] = speed_rpm;

        comprehensive_pose.motor_data.timestamp = HAL_GetTick();
        comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳
    }
}

// 获取上位机发来的电机控制命令
void GetMotorCommand(uint8_t motor_id, int8_t *direction, float *speed_rpm)
{
    if(motor_id < 4 && direction != NULL && speed_rpm != NULL) {
        // 获取互斥锁
        if (motor_cmd_mutex != NULL) {
            osMutexAcquire(motor_cmd_mutex, osWaitForever);
        }

        *direction = motor_cmd_direction[motor_id];
        *speed_rpm = motor_cmd_speed[motor_id];

        // 释放互斥锁
        if (motor_cmd_mutex != NULL) {
            osMutexRelease(motor_cmd_mutex);
        }
    }
}

// 获取上位机发来的电机命令类型
uint8_t GetMotorCommandType(uint8_t motor_id)
{
    uint8_t result = 0; // 默认返回慢刹车

    if(motor_id < 4) {
        // 获取互斥锁
        if (motor_cmd_mutex != NULL) {
            osMutexAcquire(motor_cmd_mutex, osWaitForever);
        }

        result = motor_cmd_type[motor_id];

        // 释放互斥锁
        if (motor_cmd_mutex != NULL) {
            osMutexRelease(motor_cmd_mutex);
        }
    }

    return result;
}

/* USER CODE END 4 */

/**
  * @brief  Initialize UART communication with Jetson
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef JetsonUart_Init(void)
{
    // 初始化接收缓冲区
    uart5_rx_index = 0;
    uart5_frame_complete = 0;

    // 创建电机命令数组的互斥锁
    if (motor_cmd_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "MotorCmdMutex"
        };
        motor_cmd_mutex = osMutexNew(&mutex_attr);
        if (motor_cmd_mutex == NULL) {
            return HAL_ERROR;  // 互斥锁创建失败
        }
    }

    return HAL_OK;
}

/**
  * @brief  Send heartbeat message to Jetson
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendHeartbeat(void)
{
    char heartbeat_msg[] = "HB\r\n";
    return HAL_UART_Transmit(&huart5, (uint8_t*)heartbeat_msg, strlen(heartbeat_msg), 0xFFFF);
}

/**
  * @brief  Send data message to Jetson
  * @param  data: Data to send
  * @param  length: Length of data
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendData(uint8_t *data, uint8_t length)
{
    if(length > 64) {
        return HAL_ERROR; // 数据长度超出限制
    }

    // 简单的数据发送，实际应用中可能需要添加协议头等
    return HAL_UART_Transmit(&huart5, data, length, 0xFFFF);
}

/**
  * @brief  Send acknowledgment message to Jetson
  * @param  msg_id: Message ID to acknowledge
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendAck(uint8_t msg_id)
{
    char ack_msg[20];
    sprintf(ack_msg, "ACK:%d\r\n", msg_id);
    return HAL_UART_Transmit(&huart5, (uint8_t*)ack_msg, strlen(ack_msg), 0xFFFF);
}

/**
  * @brief  Send error message to Jetson
  * @param  error_code: Error code
  * @param  error_msg: Error message string
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendError(uint8_t error_code, const char* error_msg)
{
    char err_msg[50];
    sprintf(err_msg, "ERR:%d:%s\r\n", error_code, error_msg);
    return HAL_UART_Transmit(&huart5, (uint8_t*)err_msg, strlen(err_msg), 0xFFFF);
}

/**
  * @brief  Process received data from Jetson
  * @param  data: Received data
  * @param  length: Length of received data
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_ProcessReceivedData(uint8_t *data, uint8_t length)
{
    // 处理接收到的数据
    // 这里可以实现具体的协议解析逻辑
    for(uint8_t i = 0; i < length; i++) {
        // 检查是否是换行符，表示消息结束
        if(data[i] == '\n' && uart5_rx_index > 0 && uart5_rx_buffer[uart5_rx_index-1] == '\r') {
            // 完整的消息到达 (\r\n)
            uart5_rx_buffer[uart5_rx_index] = '\0'; // 添加字符串结束符
            uart5_frame_complete = 1;

            // 尝试解析为舵机控制消息
            if(ParseServoControlMessage(uart5_rx_buffer, uart5_rx_index-2) != HAL_OK) { // -2 to exclude \r\n
                // 如果不是舵机控制消息，尝试解析为电机控制消息
                if(ParseMotorControlMessage(uart5_rx_buffer, uart5_rx_index-2) != HAL_OK) {
                    // 如果都不是，可以选择输出或忽略
                    // 对于调试，可以取消下面的注释
                    /*
                    char debug_msg[260];  // 调整缓冲区大小以适应可能的完整消息
                    sprintf(debug_msg, "Unknown command: %s", uart5_rx_buffer);
                    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
                    */
                }
            }

            // 这里可以添加消息处理逻辑
            // 例如：解析命令、执行相应操作等

            // 重置接收索引
            uart5_rx_index = 0;
        }
        // 检查是否是换行符但前面没有回车符的情况（兼容性处理）
        else if(data[i] == '\n' && uart5_rx_index > 0) {
            // 部分兼容：单独的\n也被认为是消息结束
            uart5_rx_buffer[uart5_rx_index] = '\0';
            uart5_frame_complete = 1;

            // 尝试解析为舵机控制消息
            if(ParseServoControlMessage(uart5_rx_buffer, uart5_rx_index-1) != HAL_OK) { // -1 to exclude \n
                // 如果不是舵机控制消息，尝试解析为电机控制消息
                if(ParseMotorControlMessage(uart5_rx_buffer, uart5_rx_index-1) != HAL_OK) {
                    // 如果都不是，可以选择输出或忽略
                    // 对于调试，可以取消下面的注释
                    /*
                    char debug_msg[260];
                    sprintf(debug_msg, "Unknown command: %s", uart5_rx_buffer);
                    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
                    */
                }
            }

            // 重置接收索引
            uart5_rx_index = 0;
        }
        // 检查是否是回车符，但后面可能没有换行符
        else if(data[i] == '\r' && uart5_rx_index > 0) {
            // 检查是否已经有内容，如果是则视为消息结束
            uart5_rx_buffer[uart5_rx_index] = '\0';
            uart5_frame_complete = 1;

            // 通过UART4输出接收到的数据（调试用）- 只在完整消息时输出
            char debug_msg[260];
            sprintf(debug_msg, "Message: %s", uart5_rx_buffer);
            HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);

            // 重置接收索引
            uart5_rx_index = 0;
        }
        else {
            // 检查缓冲区空间是否足够
            if(uart5_rx_index < sizeof(uart5_rx_buffer)-1) {
                // 将数据添加到接收缓冲区，但不输出
                uart5_rx_buffer[uart5_rx_index++] = data[i];
            }
            else {
                // 缓冲区溢出，重置
                uart5_rx_index = 0;
                // 添加错误信息输出
                char error_msg[] = "UART5 Buffer Overflow\r\n";
                HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0xFFFF);
            }
        }
    }

    return HAL_OK;
}

/**
  * @brief  Get pointer to receive buffer
  * @retval uint8_t*: Pointer to receive buffer
  */
uint8_t* Jetson_GetRxBuffer(void)
{
    return uart5_rx_buffer;
}

/**
  * @brief  Get length of received data
  * @retval uint8_t: Length of received data
  */
uint8_t Jetson_GetRxLength(void)
{
    return uart5_rx_index;
}

/**
  * @brief  Reset receive state
  * @retval None
  */
void Jetson_ResetRxState(void)
{
    uart5_rx_index = 0;
    uart5_frame_complete = 0;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */