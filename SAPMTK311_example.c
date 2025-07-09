/**
 * @file SAPMTK311_example.c
 * @brief SAPMTK311传感器驱动使用示例
 * @author AAA编程曹师傅 (github.com/drinktoomuchsax)
 * @date 2025-07-02
 *
 * @note 本文件展示了如何使用SAPMTK311驱动进行接近传感器和环境光传感器的操作
 */

#include "SAPMTK311.h"
#include "py32f0xx_hal.h"
#include "software_i2c.h"
#include <stdio.h>

static SAPMTK311_Handle_t h311;
static SAPMTK311_Init_t config;

/**
 * @brief 测试初始化配置（基于Archive中的PSsensor配置）
 */
void SAPMTK311_Example_TestConfig(void)
{
  printf("=== SAPMTK311 测试配置 ===\n");

  // 1. 初始化I2C总线
  I2C_Init();
  printf("I2C总线初始化完成\n");

  // 2. 创建句柄和配置
  h311.i2c_id = 0;
  h311.device_addr = SAPMTK311_I2C_ADDR;

  config = (SAPMTK311_Init_t)SAPMTK311_DEFAULT_CONFIG;
  // 基于Archive配置的测试参数
  config.enable_config.ps_enable = true;
  config.enable_config.als_enable = false;
  config.enable_config.interrupt_enable = true;

  // PS配置：0x78 = 0111 1000
  config.ps_config.gain = SAPMTK311_PS_GAIN_4X;            // 10 = 4倍增益
  config.ps_config.integration_time = SAPMTK311_PS_IT_2_96MS;
  config.ps_config.persistence = 0;                        // 00 = 无持续
  config.ps_config.threshold_high = 0x03FF;                // 1023 (0x03FF)
  config.ps_config.threshold_low = 0x02FF;                 // 767 (0x02FF)

  // LED配置：0xFF = 1111 1111
  config.led_config.drive_current = SAPMTK311_LED_IRDR_100MA; // 11 = 100mA ，实际23mA，全测
  config.led_config.duty_cycle = 63;                          // 111111 = 63 182us 中间值出发

  // 中断配置：0xA7 = 1010 01
  config.interrupt_config.interrupt_output_enable = true;

  // 等待配置：0x1F = 31
  config.wait_config.wait_time = 31;

  h311.config = config;

  // 3. 初始化传感器
  if (SAPMTK311_Init(&h311))
  {
    printf("SAPMTK311测试配置初始化成功\n");
    printf("PS阈值: 高=%d, 低=%d\n", h311.config.ps_config.threshold_high, h311.config.ps_config.threshold_low);
    printf("LED电流: 100mA, 占空比: 63\n");
  }
  else
  {
    printf("SAPMTK311测试配置初始化失败\n");
  }
}

void SAPMTK311_Example_TestPS(void)
{
  SAPMTK311_Example_TestConfig();
  SAPMTK311_Enable(&h311);
  while (1)
  {
    uint16_t ps_data;
    if (SAPMTK311_PS_GetData(&h311, &ps_data))
    {
      printf("PS数据: %d\n", ps_data);
    }
    else
    {
      printf("PS数据获取失败\n");
    }
    HAL_Delay(1000);
  }
}