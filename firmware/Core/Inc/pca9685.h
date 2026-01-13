#ifndef __PCA9685_H
#define __PCA9685_H

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* PCA9685 I2C Address */
#define PCA9685_ADDR              0x80  // 0x40 << 1
#define PCA9685_MODE1             0x00
#define PCA9685_PRESCALE          0xFE
#define PCA9685_LED0_ON_L         0x06
#define PCA9685_LED0_ON_H         0x07
#define PCA9685_LED0_OFF_L        0x08
#define PCA9685_LED0_OFF_H        0x09

/* Function declarations */
void PCA9685_Init(I2C_HandleTypeDef *hi2c, float freq);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint32_t on, uint32_t off);
void PCA9685_SetDuty(I2C_HandleTypeDef *hi2c, uint8_t num, float duty);
void PCA9685_ServoControl(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t start_angle, uint8_t end_angle, uint8_t speed);
void PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t angle);

#endif