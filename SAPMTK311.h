/**
 * @file SAPMTK311.h
 * @author AAA编程曹师傅 (github.com/drinktoomuchsax)
 * @brief SAPMTK311传感器驱动头文件
 * @version 0.2
 * @date 2025-06-30
 *
 * @copyright Copyright (c) 2025 Tangible Future
 *
 */
#ifndef __SAPMTK311_H__
#define __SAPMTK311_H__

#include "software_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Debug */
// #define SAPMTK311_DEBUG                      0

// 7-bits 从机地址 0x48
#define SAPMTK311_I2C_ADDR                   (0x48)

/* 最大支持的传感器数量 */
#define SAPMTK311_MAX_INSTANCES              4

/* ==================== 寄存器定义(总共有19个寄存器) ==================== */
/*  系统寄存器  */
#define SAPMTK311_STATE_REG                  0x00 // ALS/PS操作模式控制，等待模式控制，默认值为0x00
#define SAPMTK311_INT_REG                    0x04 // 中断引脚控制，中断持续控制，默认值为0x00
#define SAPMTK311_WAIT_REG                   0x05 // 等待时间设置，默认值为0x00
#define SAPMTK311_FLAG_REG                   0x10 // 中断标志位，默认值为0x01
#define SAPMTK311_PDT_ID_REG                 0x3E // 产品ID（只读），默认值为0x12,
#define SAPMTK311_RSRVD_REG                  0x3F // 保留寄存器，不要读写
#define SAPMTK311_SW_RESET_REG               0x80 // 软件复位（写任意值），只写
/*  PS寄存器  */
#define SAPMTK311_PSCTRL_REG                 0x01 // PS积分时间设置，PS增益设置，PS持续设置，默认值为0x31
#define SAPMTK311_LEDCTRL_REG                0x03 // LED电流设置，默认值为0xff
#define SAPMTK311_THDH1_PS_REG               0x06 // PS高阈值高位，默认值为0xff
#define SAPMTK311_THDH2_PS_REG               0x07 // PS高阈值低位，默认值为0xff
#define SAPMTK311_THDL1_PS_REG               0x08 // PS低阈值高位，默认值为0x00
#define SAPMTK311_THDL2_PS_REG               0x09 // PS低阈值低位，默认值为0x00
#define SAPMTK311_DATA1_PS_REG               0x11 // PS数据高位，默认值为0x00
#define SAPMTK311_DATA2_PS_REG               0x12 // PS数据低位，默认值为0x00
#define SAPMTK311_DATA1_OFFSET_REG           0x15 // PS偏移值高位，默认值为0x00
#define SAPMTK311_DATA2_OFFSET_REG           0x16 // PS偏移值低位，默认值为0x00
/* ALS寄存器 */
#define SAPMTK311_ALSCTRL_REG                0x02 // ALS积分时间设置，ALS增益设置，ALS持续设置，默认值为0x39
#define SAPMTK311_THDH1_ALS_REG              0x0A // ALS高阈值高位，默认值为0xff
#define SAPMTK311_THDH2_ALS_REG              0x0B // ALS高阈值低位，默认值为0xff
#define SAPMTK311_THDL1_ALS_REG              0x0C // ALS低阈值高位，默认值为0x00
#define SAPMTK311_THDL2_ALS_REG              0x0D // ALS低阈值低位，默认值为0x00
#define SAPMTK311_DATA1_ALS_REG              0x13 // ALS数据高位，默认值为0x00
#define SAPMTK311_DATA2_ALS_REG              0x14 // ALS数据低位，默认值为0x00
/* IR寄存器 */
#define SAPMTK311_DATA1_IR_REG               0x17 // IR数据高位，默认值为0x00
#define SAPMTK311_DATA2_IR_REG               0x18 // IR数据低位，默认值为0x00

/* 状态寄存器位定义 */
#define SAPMTK311_STATE_EN_IRS_SHIFT         7 // IR传感器使能
#define SAPMTK311_STATE_EN_AK_SHIFT          6 // ALS/PS传感器使能
#define SAPMTK311_STATE_EN_ASO_SHIFT         5 // ADC减去偏移使能（Adc Substract Offset）
#define SAPMTK311_STATE_EN_IRO_SHIFT         4 // Interuput run once 模式使能
#define SAPMTK311_STATE_EN_WAIT_SHIFT        2 // 等待使能
#define SAPMTK311_STATE_EN_ALS_SHIFT         1 // ALS使能
#define SAPMTK311_STATE_EN_PS_SHIFT          0 // PS使能

#define SAPMTK311_STATE_EN_IRS_MASK          (0x01U << SAPMTK311_STATE_EN_IRS_SHIFT)
#define SAPMTK311_STATE_EN_AK_MASK           (0x01U << SAPMTK311_STATE_EN_AK_SHIFT)
#define SAPMTK311_STATE_EN_ASO_MASK          (0x01U << SAPMTK311_STATE_EN_ASO_SHIFT)
#define SAPMTK311_STATE_EN_IRO_MASK          (0x01U << SAPMTK311_STATE_EN_IRO_SHIFT)
#define SAPMTK311_STATE_EN_WAIT_MASK         (0x01U << SAPMTK311_STATE_EN_WAIT_SHIFT)
#define SAPMTK311_STATE_EN_ALS_MASK          (0x01U << SAPMTK311_STATE_EN_ALS_SHIFT)
#define SAPMTK311_STATE_EN_PS_MASK           (0x01U << SAPMTK311_STATE_EN_PS_SHIFT)

/* PS控制寄存器位定义 */
#define SAPMTK311_PS_PRS_SHIFT               6
#define SAPMTK311_PS_GAIN_SHIFT              4
#define SAPMTK311_PS_IT_SHIFT                0

#define SAPMTK311_PS_PRS_MASK                0xC0
#define SAPMTK311_PS_GAIN_MASK               0x30
#define SAPMTK311_PS_IT_MASK                 0x0F

/* ALS控制寄存器位定义 */
#define SAPMTK311_ALS_PRS_SHIFT              6
#define SAPMTK311_ALS_GAIN_SHIFT             4
#define SAPMTK311_ALS_IT_SHIFT               0

#define SAPMTK311_ALS_PRS_MASK               0xC0
#define SAPMTK311_ALS_GAIN_MASK              0x30
#define SAPMTK311_ALS_IT_MASK                0x0F

/* LED控制寄存器位定义 */
#define SAPMTK311_LED_IRDR_SHIFT             6
#define SAPMTK311_LED_DT_SHIFT               0

#define SAPMTK311_LED_IRDR_MASK              0xC0
#define SAPMTK311_LED_DT_MASK                0x3F

/* 中断寄存器位定义 */
#define SAPMTK311_INT_CTRL_SHIFT             7
#define SAPMTK311_INT_OUI_SHIFT              4
#define SAPMTK311_INT_ALS_SHIFT              3
#define SAPMTK311_INT_PS_SHIFT               0

#define SAPMTK311_INT_CTRL_MASK              0x80
#define SAPMTK311_INT_OUI_MASK               0x10
#define SAPMTK311_INT_ALS_MASK               0x08
#define SAPMTK311_INT_PS_MASK                0x07

/* 标志寄存器位定义 */
#define SAPMTK311_FLG_ALSDR_SHIFT            7
#define SAPMTK311_FLG_PSDR_SHIFT             6
#define SAPMTK311_FLG_ALSINT_SHIFT           5
#define SAPMTK311_FLG_PSINT_SHIFT            4
#define SAPMTK311_FLG_OUI_SHIFT              2
#define SAPMTK311_FLG_IR_RDY_SHIFT           1
#define SAPMTK311_FLG_NF_SHIFT               0

#define SAPMTK311_FLG_ALSDR_MASK             0x80
#define SAPMTK311_FLG_PSDR_MASK              0x40
#define SAPMTK311_FLG_ALSINT_MASK            0x20
#define SAPMTK311_FLG_PSINT_MASK             0x10
#define SAPMTK311_FLG_OUI_MASK               0x04
#define SAPMTK311_FLG_IR_RDY_MASK            0x02
#define SAPMTK311_FLG_NF_MASK                0x01

/* 产品ID定义 */
#define SAPMTK311_PID                        0x12

/* 默认配置参数 */
#define SAPMTK311_DEFAULT_PS_THRESHOLD_HIGH  1000
#define SAPMTK311_DEFAULT_PS_THRESHOLD_LOW   800
#define SAPMTK311_DEFAULT_ALS_THRESHOLD_HIGH 1000
#define SAPMTK311_DEFAULT_ALS_THRESHOLD_LOW  100

/* 传感器状态定义 */
typedef enum
{
  SAPMTK311_STATE_FAR = 0,  // 远离状态
  SAPMTK311_STATE_NEAR = 1, // 接近状态
  SAPMTK311_STATE_ERROR = 2 // 错误状态
} SAPMTK311_PS_State_t;

/* PS增益设置 */
typedef enum
{
  SAPMTK311_PS_GAIN_1X = 0,  // 1倍增益
  SAPMTK311_PS_GAIN_4X = 1,  // 4倍增益
  SAPMTK311_PS_GAIN_16X = 2, // 16倍增益
  SAPMTK311_PS_GAIN_64X = 3  // 64倍增益
} SAPMTK311_PS_Gain_t;

/* PS积分时间设置，0.185ms*(2^X),X=0~15 */
typedef enum
{
  SAPMTK311_PS_IT_0_185MS = 0,    // 0.185ms
  SAPMTK311_PS_IT_0_37MS = 1,     // 0.37ms
  SAPMTK311_PS_IT_0_74MS = 2,     // 0.74ms
  SAPMTK311_PS_IT_1_48MS = 3,     // 1.48ms
  SAPMTK311_PS_IT_2_96MS = 4,     // 2.96ms
  SAPMTK311_PS_IT_5_92MS = 5,     // 5.92ms
  SAPMTK311_PS_IT_11_84MS = 6,    // 11.84ms
  SAPMTK311_PS_IT_23_68MS = 7,    // 23.68ms
  SAPMTK311_PS_IT_47_36MS = 8,    // 47.36ms
  SAPMTK311_PS_IT_94_72MS = 9,    // 94.72ms
  SAPMTK311_PS_IT_189_44MS = 10,  // 189.44ms
  SAPMTK311_PS_IT_378_88MS = 11,  // 378.88ms
  SAPMTK311_PS_IT_757_76MS = 12,  // 757.76ms
  SAPMTK311_PS_IT_1515_52MS = 13, // 1515.52ms 1.515s
  SAPMTK311_PS_IT_3031_04MS = 14, // 3031.04ms 3.031s
  SAPMTK311_PS_IT_6062_08MS = 15  // 6062.08ms 6.062s
} SAPMTK311_PS_IT_t;

/* ALS增益设置 */
typedef enum
{
  SAPMTK311_ALS_GAIN_1X = 0,  // 1倍增益
  SAPMTK311_ALS_GAIN_4X = 1,  // 4倍增益
  SAPMTK311_ALS_GAIN_16X = 2, // 16倍增益
  SAPMTK311_ALS_GAIN_64X = 3  // 64倍增益
} SAPMTK311_ALS_Gain_t;

/* ALS积分时间设置 */
typedef enum
{
  SAPMTK311_ALS_IT_0_185MS = 0,    // 0.185ms
  SAPMTK311_ALS_IT_0_37MS = 1,     // 0.37ms
  SAPMTK311_ALS_IT_0_74MS = 2,     // 0.74ms
  SAPMTK311_ALS_IT_1_48MS = 3,     // 1.48ms
  SAPMTK311_ALS_IT_2_96MS = 4,     // 2.96ms
  SAPMTK311_ALS_IT_5_92MS = 5,     // 5.92ms
  SAPMTK311_ALS_IT_11_84MS = 6,    // 11.84ms
  SAPMTK311_ALS_IT_23_68MS = 7,    // 23.68ms
  SAPMTK311_ALS_IT_47_36MS = 8,    // 47.36ms
  SAPMTK311_ALS_IT_94_72MS = 9,    // 94.72ms
  SAPMTK311_ALS_IT_189_44MS = 10,  // 189.44ms
  SAPMTK311_ALS_IT_378_88MS = 11,  // 378.88ms
  SAPMTK311_ALS_IT_757_76MS = 12,  // 757.76ms
  SAPMTK311_ALS_IT_1515_52MS = 13, // 1515.52ms 1.515s
  SAPMTK311_ALS_IT_3031_04MS = 14, // 3031.04ms 3.031s
  SAPMTK311_ALS_IT_6062_08MS = 15  // 6062.08ms 6.062s
} SAPMTK311_ALS_IT_t;

/* LED驱动电流设置 */
typedef enum
{
  SAPMTK311_LED_IRDR_12_5MA = 0, // 12.5mA
  SAPMTK311_LED_IRDR_25MA = 1,   // 25mA
  SAPMTK311_LED_IRDR_50MA = 2,   // 50mA
  SAPMTK311_LED_IRDR_100MA = 3   // 100mA
} SAPMTK311_LED_IRDR_t;

/* 初始化配置结构体 - 类似HAL库GPIO_InitTypeDef的方式设计 */
typedef struct
{
  // 功能使能配置
  struct
  {
    bool ps_enable;           // PS功能使能
    bool als_enable;          // ALS功能使能
    bool wait_enable;         // 等待模式使能
    bool interrupt_run_once;  // 中断运行一次模式使能
    bool adc_subtract_offset; // ADC减去偏移使能
    bool ir_enable;           // IR功能使能
    bool interrupt_enable;    // 中断使能
  } enable_config;

  // PS配置
  struct
  {
    SAPMTK311_PS_Gain_t gain;           // PS增益设置
    SAPMTK311_PS_IT_t integration_time; // PS积分时间
    uint8_t persistence;                // PS持续设置
    uint16_t threshold_high;            // PS高阈值
    uint16_t threshold_low;             // PS低阈值
    bool auto_calibration;              // 自动校准使能
  } ps_config;

  // ALS配置
  struct
  {
    SAPMTK311_ALS_Gain_t gain;           // ALS增益设置
    SAPMTK311_ALS_IT_t integration_time; // ALS积分时间
    uint8_t persistence;                 // ALS持续设置
    uint16_t threshold_high;             // ALS高阈值
    uint16_t threshold_low;              // ALS低阈值
    bool auto_calibration;               // 自动校准使能
  } als_config;

  // LED配置
  struct
  {
    SAPMTK311_LED_IRDR_t drive_current; // LED驱动电流
    uint8_t duty_cycle;                 // LED占空比 duty_cycle/64 * 0.185ms
  } led_config;

  // 中断配置
  struct
  {
    uint8_t ps_mode;               // PS中断模式 (0-7)
    uint8_t als_mode;              // ALS中断模式
    uint8_t interrupt_persistence; // 中断持续设置
    bool interrupt_output_enable;  // 中断输出使能
  } interrupt_config;

  // 等待配置
  struct
  {
    uint8_t wait_time; // 等待时间设置
  } wait_config;

  // 电源管理配置
  struct
  {
    bool low_power_mode; // 低功耗模式
    uint8_t power_mode;  // 电源模式
  } power_config;

} SAPMTK311_Init_t;
/* 传感器DATA结构体 */
typedef struct
{
  uint16_t ps_data;              // PS读数，16位，无量纲
  uint16_t als_data;             // ALS读数，16位，单位为lux
  SAPMTK311_PS_State_t ps_state; // PS状态
  uint16_t ps_dynamic_threshold; // PS动态阈值
} SAPMTK311_Data_t;

/* 传感器句柄结构体 - 类似HAL库的句柄设计 */
typedef struct
{
  // 当前配置（运行时可能被修改）
  SAPMTK311_Init_t config; // 当前配置
  bool is_initialized;     // 初始化标志

  // 硬件相关
  uint8_t i2c_id;             // I2C总线ID
  uint8_t device_addr;        // 设备地址
  GPIO_TypeDef *GPIOX;        // 悬崖检测GPIO
  uint16_t GPIO_PIN_X;        // 输出悬崖检测结果的GPIO
  uint16_t dynamic_threshold; // 动态阈值

  // 运行时状态
  bool ps_enabled;       // PS使能标志
  bool als_enabled;      // ALS使能标志
  SAPMTK311_Data_t data; // 传感器数据

} SAPMTK311_Handle_t;

/* 默认初始化配置 */
#define SAPMTK311_DEFAULT_CONFIG {                              \
    .enable_config = {                                          \
        .ps_enable = true,                                      \
        .als_enable = true,                                     \
        .ir_enable = false,                                     \
        .wait_enable = true,                                    \
        .interrupt_enable = false,                              \
        .adc_subtract_offset = false,                           \
        .interrupt_run_once = false,                            \
    },                                                          \
    .ps_config = {                                              \
        .gain = SAPMTK311_PS_GAIN_1X,                           \
        .integration_time = SAPMTK311_PS_IT_2_96MS,             \
        .persistence = 0,                                       \
        .threshold_high = SAPMTK311_DEFAULT_PS_THRESHOLD_HIGH,  \
        .threshold_low = SAPMTK311_DEFAULT_PS_THRESHOLD_LOW,    \
        .auto_calibration = false,                              \
    },                                                          \
    .als_config = {                                             \
        .gain = SAPMTK311_ALS_GAIN_1X,                          \
        .integration_time = SAPMTK311_ALS_IT_189_44MS,          \
        .persistence = 0,                                       \
        .threshold_high = SAPMTK311_DEFAULT_ALS_THRESHOLD_HIGH, \
        .threshold_low = SAPMTK311_DEFAULT_ALS_THRESHOLD_LOW,   \
        .auto_calibration = false,                              \
    },                                                          \
    .led_config = {                                             \
        .drive_current = SAPMTK311_LED_IRDR_25MA,               \
        .duty_cycle = 0x20,                                     \
    },                                                          \
    .interrupt_config = {                                       \
        .ps_mode = 0x01,                                        \
        .als_mode = 0x00,                                       \
        .interrupt_persistence = 0,                             \
        .interrupt_output_enable = false,                       \
    },                                                          \
    .wait_config = {                                            \
        .wait_time = 0,                                         \
    },                                                          \
    .power_config = {                                           \
        .low_power_mode = false,                                \
        .power_mode = 0,                                        \
    },                                                          \
}

/* 快速配置宏定义 */
// PS接近检测配置
#define SAPMTK311_PS_PROXIMITY_CONFIG {                \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = true,                      \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_4X,                  \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 800,                         \
        .threshold_low = 600,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 1000,                        \
        .threshold_low = 100,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_50MA,      \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x01,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = true,               \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}

/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Apple {                  \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_50MA,      \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Bike {                   \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_12_5MA,    \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Car {                    \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_25MA,      \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Dude {                   \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_100MA,     \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Fun {                    \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_64X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_50MA,      \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Game {                   \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_50MA,      \
        .duty_cycle = 0x40,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 测试配置宏定义- */
#define SAPMTK311_TEST_CONFIG_Home {                   \
    .enable_config = {                                 \
        .ps_enable = true,                             \
        .als_enable = false,                           \
        .ir_enable = false,                            \
        .wait_enable = false,                          \
        .interrupt_enable = false,                     \
        .adc_subtract_offset = false,                  \
        .interrupt_run_once = false,                   \
    },                                                 \
    .ps_config = {                                     \
        .gain = SAPMTK311_PS_GAIN_16X,                 \
        .integration_time = SAPMTK311_PS_IT_2_96MS,    \
        .persistence = 1,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = true,                      \
    },                                                 \
    .als_config = {                                    \
        .gain = SAPMTK311_ALS_GAIN_1X,                 \
        .integration_time = SAPMTK311_ALS_IT_189_44MS, \
        .persistence = 0,                              \
        .threshold_high = 888,                         \
        .threshold_low = 666,                          \
        .auto_calibration = false,                     \
    },                                                 \
    .led_config = {                                    \
        .drive_current = SAPMTK311_LED_IRDR_50MA,      \
        .duty_cycle = 0x30,                            \
    },                                                 \
    .interrupt_config = {                              \
        .ps_mode = 0x00,                               \
        .als_mode = 0x00,                              \
        .interrupt_persistence = 0,                    \
        .interrupt_output_enable = false,              \
    },                                                 \
    .wait_config = {                                   \
        .wait_time = 0,                                \
    },                                                 \
    .power_config = {                                  \
        .low_power_mode = false,                       \
        .power_mode = 0,                               \
    },                                                 \
}
/* 函数声明 */

// 基础I2C操作函数
bool SAPMTK311_WriteByte(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t data);
bool SAPMTK311_WriteBytes(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len);
bool SAPMTK311_ReadByte(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data);
bool SAPMTK311_ReadBytes(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len);

// 初始化函数
bool SAPMTK311_Init(SAPMTK311_Handle_t *handle);
bool SAPMTK311_DeInit(SAPMTK311_Handle_t *handle);
bool SAPMTK311_Reset(SAPMTK311_Handle_t *handle);
bool SAPMTK311_CheckID(SAPMTK311_Handle_t *handle);
bool SAPMTK311_Enable(SAPMTK311_Handle_t *handle);

// 配置管理函数
bool SAPMTK311_ApplyConfig(SAPMTK311_Handle_t *handle, const SAPMTK311_Init_t *config);
bool SAPMTK311_GetConfig(SAPMTK311_Handle_t *handle, SAPMTK311_Init_t *config);

// PS相关函数
bool SAPMTK311_PS_SetThreshold(SAPMTK311_Handle_t *handle, uint16_t high, uint16_t low);
bool SAPMTK311_PS_GetData(SAPMTK311_Handle_t *handle, uint16_t *data);
SAPMTK311_PS_State_t SAPMTK311_PS_GetState(SAPMTK311_Handle_t *handle, SAPMTK311_PS_State_t *state);

// ALS相关函数
bool SAPMTK311_ALS_SetThreshold(SAPMTK311_Handle_t *handle, uint16_t high, uint16_t low);
bool SAPMTK311_ALS_GetData(SAPMTK311_Handle_t *handle, uint16_t *data);

// LED相关函数
bool SAPMTK311_LED_SetIRDR(SAPMTK311_Handle_t *handle, SAPMTK311_LED_IRDR_t irdr);
bool SAPMTK311_LED_SetDutyCycle(SAPMTK311_Handle_t *handle, uint8_t duty_cycle);

// 中断相关函数

// 状态查询函数
bool SAPMTK311_GetFlag(SAPMTK311_Handle_t *handle, uint8_t *flag);
bool SAPMTK311_IsDataReady(SAPMTK311_Handle_t *handle);

#endif /* __SAPMTK311_H__ */
