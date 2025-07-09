/**
 * @file SAPMTK311_Array.c
 * @author AAA编程曹师傅 (github.com/drinktoomuchsax)
 * @brief 这个文件使用4个SAPMTK311传感器阵列，时分复用，实现PS数据的采集
 * @version 0.1
 * @date 2025-07-02
 *
 * @copyright Copyright (c) 2025 Tangible Future
 *
 */

#include "SAPMTK311_Array.h"
#include "py32f0xx_hal.h"
#include "software_i2c.h"
#include <stdio.h>

#define TASK_TYPE TASK_OUTPUTS_BASED_ON_THRESHOLDS

SAPMTK311_Handle_t SAPMTK311_Array[SENSOR_ARRAY_SIZE];

static void handle_get_characteristic_curve(void);
static void handle_outputs_based_on_thresholds(void);

/**
 * @brief 初始化4个SAPMTK311传感器
 *
 * @note 使用不同的I2C总线ID来区分4个传感器
 */
void SAPMTK311_Array_Init(void)
{
  printf("=== SAPMTK311 传感器阵列初始化 ===\n");

  // 初始化I2C总线
  I2C_Init();
  printf("I2C总线初始化完成\n");

  // 初始化4个IIC
  for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
  {
    // 创建句柄和配置
    SAPMTK311_Array[i].i2c_id = i; // 使用不同的I2C总线ID
    SAPMTK311_Array[i].device_addr = SAPMTK311_I2C_ADDR;

    // 使用默认配置
    SAPMTK311_Array[i].config = (SAPMTK311_Init_t)SAPMTK311_TEST_CONFIG_Apple;

    // 3. 初始化传感器
    if (SAPMTK311_Init(&SAPMTK311_Array[i]))
    {
      printf("传感器 %d 初始化成功\n", i);

      // 4. 使能传感器
      if (SAPMTK311_Enable(&SAPMTK311_Array[i]))
      {
        printf("传感器 %d 使能成功\n", i);
      }
      else
      {
        printf("传感器 %d 使能失败\n", i);
      }
    }
    else
    {
      printf("传感器 %d 初始化失败\n", i);
    }
    // 延时，避免I2C冲突
    HAL_Delay(5);
  }

  // 初始化GPIO和动态阈值
  for (size_t i = 0; i < SENSOR_ARRAY_SIZE; i++)
  {
    SAPMTK311_Array[i].GPIOX = GPIOA;
    SAPMTK311_Array[i].dynamic_threshold = DEFAULT_THRESHOLD;
  }

  // 初始化GPIO，顺序在PL2219中亦有记载
  SAPMTK311_Array[SENSOR_FRONT_LEFT].GPIO_PIN_X = GPIO_PIN_5;
  SAPMTK311_Array[SENSOR_FRONT_RIGHT].GPIO_PIN_X = GPIO_PIN_1;
  SAPMTK311_Array[SENSOR_BACK_LEFT].GPIO_PIN_X = GPIO_PIN_6;
  SAPMTK311_Array[SENSOR_BACK_RIGHT].GPIO_PIN_X = GPIO_PIN_7;

  printf("=== SAPMTK311 传感器阵列初始化完成 ===\n");
}

/**
 * @brief 传感器阵列任务函数
 *
 * @note 时分复用读取4个传感器的数据
 */
void SAPMTK311_Array_Task(void)
{
  switch (TASK_TYPE)
  {
  case TASK_GET_CHARACTERISTIC_CURVE:
    handle_get_characteristic_curve();
    break;
  case TASK_OUTPUTS_BASED_ON_THRESHOLDS:
    handle_outputs_based_on_thresholds();
    break;
  }
}

/**
 * @brief 根据阈值输出高低电平
 *
 * @note 根据底盘四个接近传感器的读数和阈值来比较，在四个gpio上输出高低电平（高电平表示悬崖，低电平表示桌面）
 */
static void handle_outputs_based_on_thresholds(void)
{
  while (1)
  {
    for (size_t i = 0; i < SENSOR_ARRAY_SIZE; i++)
    {
      uint16_t ps_data;
      if (SAPMTK311_PS_GetData(&SAPMTK311_Array[i], &ps_data))
      {
        if (ps_data < SAPMTK311_Array[i].dynamic_threshold)
        { // 如果读数小于动态阈值，则输出高电平
          HAL_GPIO_WritePin(SAPMTK311_Array[i].GPIOX, SAPMTK311_Array[i].GPIO_PIN_X, GPIO_PIN_SET);
        }
        else
        { // 如果读数大于动态阈值，则输出低电平
          HAL_GPIO_WritePin(SAPMTK311_Array[i].GPIOX, SAPMTK311_Array[i].GPIO_PIN_X, GPIO_PIN_RESET);
        }
      }
    }
  }
}

/**
 * @brief 获取传感器“读数-距离”特征曲线
 *
 * @note 用来跑传感器的特征曲线，配合python脚本上位机使用
 */
static void handle_get_characteristic_curve(void)
{
  uint16_t ps_data;
  for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
  {
    // 三次求平均PS_data
    for (int j = 0; j < 3; j++)
    {
      if (SAPMTK311_PS_GetData(&SAPMTK311_Array[i], &ps_data))
      {
        ps_data += ps_data;
      }
      else
      {
        printf("传感器 %d PS数据获取失败\n", i);
      }
      HAL_Delay(5);
    }
    ps_data /= 3;
    printf("传感器 %d PS数据: %d\n", i, ps_data);

    // 检查接近状态
    SAPMTK311_PS_State_t state;
    if (SAPMTK311_PS_GetState(&SAPMTK311_Array[i], &state) == SAPMTK311_STATE_NEAR)
    {
      printf("传感器 %d 检测到接近\n", i);
    }
    HAL_Delay(5);
  }
}

void SAPMTK311_Array_Calibrate_Proximity_10Times_Average(uint32_t *calibrated_proximity)
{
  uint16_t ps_data;
  for (int i = 0; i < SENSOR_ARRAY_SIZE; i++)
  {
    for (int j = 0; j < 10; j++)
    {
      if (SAPMTK311_PS_GetData(&SAPMTK311_Array[i], &ps_data))
      {
        calibrated_proximity[i] += ps_data;
      }
    }
    calibrated_proximity[i] /= 10;
  }
}