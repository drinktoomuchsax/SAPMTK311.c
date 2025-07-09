/**
 * @file SAPMTK311_Array.h
 * @author AAA编程曹师傅 (github.com/drinktoomuchsax)
 * @brief SAPMTK311传感器阵列头文件
 * @version 0.1
 * @date 2025-07-02
 *
 * @copyright Copyright (c) 2025 Tangible Future
 *
 */
#ifndef __SAPMTK311_ARRAY_H__
#define __SAPMTK311_ARRAY_H__

#include "SAPMTK311.h"

// SAPMTK311传感器位置枚举
typedef enum
{
  SENSOR_FRONT_LEFT = 0, // 前左
  SENSOR_FRONT_RIGHT,    // 前右
  SENSOR_BACK_LEFT,      // 后左
  SENSOR_BACK_RIGHT,     // 后右
  SENSOR_ARRAY_SIZE
} SensorPosition_t;

typedef enum
{
  TASK_GET_CHARACTERISTIC_CURVE,
  TASK_OUTPUTS_BASED_ON_THRESHOLDS,
  TASK_ARRAY_SIZE
} TASK_TYPE_t;

// GPIO引脚定义
#define DEFAULT_THRESHOLD 200

extern SAPMTK311_Handle_t SAPMTK311_Array[SENSOR_ARRAY_SIZE];

void SAPMTK311_Array_Init(void);
void SAPMTK311_Array_Task(void);
void SAPMTK311_Array_Calibrate_Proximity_10Times_Average(uint32_t *calibrated_proximity);

#endif