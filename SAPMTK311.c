/**
 * @file SAPMTK311.c
 * @author AAA编程曹师傅 (github.com/drinktoomuchsax)
 * @brief SAPMTK311传感器驱动 - HAL风格实现
 * @version 0.3
 * @date 2025-06-30
 *
 * @copyright Copyright (c) 2025 Tangible Future
 *
 */

#include "SAPMTK311.h"
#include "py32f0xx_hal.h"

/**
 * @brief 写入单个字节到指定寄存器
 * @param handle 传感器句柄
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_WriteByte(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t data)
{
  if (!handle)
  {
    return false;
  }

  return I2C_Write(handle->i2c_id, handle->device_addr, reg, &data, 1);
}

/**
 * @brief 写入多个字节到指定寄存器
 * @param handle 传感器句柄
 * @param reg 寄存器地址
 * @param data 数据数组
 * @param len 数据长度
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_WriteBytes(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len)
{
  if (!handle || data == NULL)
  {
    return false;
  }

  return I2C_Write(handle->i2c_id, handle->device_addr, reg, data, len);
}

/**
 * @brief 从指定寄存器读取单个字节
 * @param handle 传感器句柄
 * @param reg 寄存器地址
 * @param data 读取的数据指针
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_ReadByte(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data)
{
  if (!handle || data == NULL)
  {
    return false;
  }

  return I2C_Read(handle->i2c_id, handle->device_addr, reg, data, 1);
}

/**
 * @brief 从指定寄存器读取多个字节
 * @param handle 传感器句柄
 * @param reg 寄存器地址
 * @param data 数据数组指针
 * @param len 要读取的长度
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_ReadBytes(SAPMTK311_Handle_t *handle, uint8_t reg, uint8_t *data, uint8_t len)
{
  if (!handle || data == NULL)
  {
    return false;
  }

  return I2C_Read(handle->i2c_id, handle->device_addr, reg, data, len);
}

/**
 * @brief 初始化SAPMTK311传感器
 * @param handle 传感器句柄（必须预先配置好config）
 * @return 成功返回true，失败返回false
 * @note 初始化包含以下步骤：检查配置是否有效 -> 检查设备ID -> 软件复位 -> 等待复位完成 -> 应用配置 -> 设置初始化标志
 */
bool SAPMTK311_Init(SAPMTK311_Handle_t *handle)
{
  if (!handle)
  {
    return false;
  }

  // 检查配置是否有效
  if (handle->i2c_id == 0 && handle->device_addr == 0)
  {
    return false;
  }

  handle->is_initialized = false;

  // 检查设备ID
  if (!SAPMTK311_CheckID(handle))
  {
#if SAPMTK311_DEBUG
    printf("SAPMTK311: Device ID check failed\n");
#endif
    return false;
  }

  // 软件复位
  if (!SAPMTK311_Reset(handle))
  {
#if SAPMTK311_DEBUG
    printf("SAPMTK311: Software reset failed\n");
#endif
    return false;
  }

  // 等待复位完成
  HAL_Delay(10);

  // 应用配置
  if (!SAPMTK311_ApplyConfig(handle, &handle->config))
  {
#if SAPMTK311_DEBUG
    printf("SAPMTK311: Apply configuration failed\n");
#endif
    return false;
  }

  handle->is_initialized = true;

#if SAPMTK311_DEBUG
  printf("SAPMTK311: Initialization completed successfully\n");
#endif

  return true;
}

/**
 * @brief 反初始化SAPMTK311传感器
 * @param handle 传感器句柄
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_DeInit(SAPMTK311_Handle_t *handle)
{
  if (!handle || !handle->is_initialized)
  {
    return false;
  }

  // 禁用PS和ALS
  // SAPMTK311_PS_Enable(handle, false);
  // SAPMTK311_ALS_Enable(handle, false);

  // 禁用中断
  // SAPMTK311_Interrupt_Enable(handle, false);

  handle->is_initialized = false;

  return true;
}

/**
 * @brief 软件复位
 * @param handle 传感器句柄
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_Reset(SAPMTK311_Handle_t *handle)
{
  if (!handle)
  {
    return false;
  }

  return SAPMTK311_WriteByte(handle, SAPMTK311_SW_RESET_REG, 0x00);
}

/**
 * @brief 检查设备ID
 * @param handle 传感器句柄
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_CheckID(SAPMTK311_Handle_t *handle)
{
  if (!handle)
  {
    return false;
  }

  uint8_t pid = 66;

  if (!SAPMTK311_ReadByte(handle, SAPMTK311_PDT_ID_REG, &pid))
  {
#if SAPMTK311_DEBUG
    printf("SAPMTK311_ReadByte failed\r\n");
#endif
    return false;
  }

#if SAPMTK311_DEBUG
  printf("SAPMTK311_ReadByte success\r\n");
  printf("pid: %d\r\n", pid);
#endif

  return (pid == SAPMTK311_PID);
}

/**
 * @brief 应用配置到传感器
 * @param handle 传感器句柄
 * @param config 配置结构体指针
 * @return 成功返回true，失败返回false
 * @note 应用配置包含以下步骤：检查配置是否有效 -> 配置STATE寄存器 -> 配置PS控制寄存器 -> 设置PS阈值 -> 配置ALS控制寄存器 -> 设置ALS阈值 -> 配置LED控制寄存器 -> 配置中断寄存器 -> 配置等待寄存器 -> 更新内部状态
 */
bool SAPMTK311_ApplyConfig(SAPMTK311_Handle_t *handle, const SAPMTK311_Init_t *config)
{
  if (!handle || !config)
  {
    return false;
  }
#ifdef SAPMTK311_DEBUG
  printf("SAPMTK311_ApplyConfig\r\n");
#endif
  // 配置状态寄存器
  uint8_t state_reg = 0x00;

  if (config->enable_config.ps_enable)
    state_reg |= SAPMTK311_STATE_EN_PS_MASK;
  if (config->enable_config.als_enable)
    state_reg |= SAPMTK311_STATE_EN_ALS_MASK;
  if (config->enable_config.ir_enable)
    state_reg |= SAPMTK311_STATE_EN_IRS_MASK;
  if (config->enable_config.wait_enable)
    state_reg |= SAPMTK311_STATE_EN_WAIT_MASK;
  if (config->enable_config.adc_subtract_offset)
    state_reg |= SAPMTK311_STATE_EN_ASO_MASK;
  if (config->enable_config.interrupt_run_once)
    state_reg |= SAPMTK311_STATE_EN_IRO_MASK;

  if (!SAPMTK311_WriteByte(handle, SAPMTK311_STATE_REG, state_reg))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteByte failed\r\n");
#endif
    return false;
  }

  // 配置PS控制寄存器
  if (config->enable_config.ps_enable)
  {
    uint8_t ps_ctrl = (config->ps_config.gain << SAPMTK311_PS_GAIN_SHIFT) |
                      (config->ps_config.integration_time & SAPMTK311_PS_IT_MASK) |
                      ((config->ps_config.persistence & 0x03) << SAPMTK311_PS_PRS_SHIFT);

    if (!SAPMTK311_WriteByte(handle, SAPMTK311_PSCTRL_REG, ps_ctrl))
    {
#ifdef SAPMTK311_DEBUG
      printf("SAPMTK311_WriteByte failed\r\n");
#endif
      return false;
    }

    // 设置PS阈值
    if (!SAPMTK311_PS_SetThreshold(handle, config->ps_config.threshold_high, config->ps_config.threshold_low))
    {
#ifdef SAPMTK311_DEBUG
      printf("SAPMTK311_PS_SetThreshold failed\r\n");
#endif
      return false;
    }
  }

  // 配置ALS控制寄存器
  if (config->enable_config.als_enable)
  {
    uint8_t als_ctrl = (config->als_config.gain << SAPMTK311_ALS_GAIN_SHIFT) |
                       (config->als_config.integration_time & SAPMTK311_ALS_IT_MASK) |
                       ((config->als_config.persistence & 0x03) << SAPMTK311_ALS_PRS_SHIFT);

    if (!SAPMTK311_WriteByte(handle, SAPMTK311_ALSCTRL_REG, als_ctrl))
    {
#ifdef SAPMTK311_DEBUG
      printf("SAPMTK311_WriteByte failed\r\n");
#endif
      return false;
    }

    // 设置ALS阈值
    if (!SAPMTK311_ALS_SetThreshold(handle, config->als_config.threshold_high, config->als_config.threshold_low))
    {
#ifdef SAPMTK311_DEBUG
      printf("SAPMTK311_ALS_SetThreshold failed\r\n");
#endif
      return false;
    }
  }

  // 配置LED控制寄存器
  uint8_t led_ctrl = (config->led_config.drive_current << SAPMTK311_LED_IRDR_SHIFT) |
                     (config->led_config.duty_cycle & SAPMTK311_LED_DT_MASK);

  if (!SAPMTK311_WriteByte(handle, SAPMTK311_LEDCTRL_REG, led_ctrl))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteByte failed\r\n");
#endif
    return false;
  }

  // 配置中断寄存器
  uint8_t int_reg = 0x00;

  if (config->enable_config.interrupt_enable)
    int_reg |= SAPMTK311_INT_CTRL_MASK;
  if (config->interrupt_config.interrupt_output_enable)
    int_reg |= SAPMTK311_INT_OUI_MASK;

  int_reg |= (config->interrupt_config.ps_mode & SAPMTK311_INT_PS_MASK);
  int_reg |= ((config->interrupt_config.als_mode & 0x01) << SAPMTK311_INT_ALS_SHIFT);

  if (!SAPMTK311_WriteByte(handle, SAPMTK311_INT_REG, int_reg))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteByte failed\r\n");
#endif
    return false;
  }

  // 配置等待寄存器
  if (config->enable_config.wait_enable)
  {
    if (!SAPMTK311_WriteByte(handle, SAPMTK311_WAIT_REG, config->wait_config.wait_time))
    {
#ifdef SAPMTK311_DEBUG
      printf("SAPMTK311_WriteByte failed\r\n");
#endif
      return false;
    }
  }

  // 更新内部状态
  handle->ps_enabled = config->enable_config.ps_enable;
  handle->als_enabled = config->enable_config.als_enable;

  return true;
}

/**
 * @brief 获取当前配置
 * @param handle 传感器句柄
 * @param config 配置结构体指针
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_GetConfig(SAPMTK311_Handle_t *handle, SAPMTK311_Init_t *config)
{
  if (!config || !handle || !handle->is_initialized)
  {
    return false;
  }

  // 读取状态寄存器
  uint8_t state_reg;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_STATE_REG, &state_reg))
  {
    return false;
  }

  // 读取PS控制寄存器
  uint8_t ps_ctrl;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_PSCTRL_REG, &ps_ctrl))
  {
    return false;
  }

  // 读取ALS控制寄存器
  uint8_t als_ctrl;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_ALSCTRL_REG, &als_ctrl))
  {
    return false;
  }

  // 读取LED控制寄存器
  uint8_t led_ctrl;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_LEDCTRL_REG, &led_ctrl))
  {
    return false;
  }

  // 读取中断寄存器
  uint8_t int_reg;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_INT_REG, &int_reg))
  {
    return false;
  }

  // 读取等待寄存器
  uint8_t wait_reg;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_WAIT_REG, &wait_reg))
  {
    return false;
  }

  // 功能使能配置
  config->enable_config.ps_enable = (state_reg & SAPMTK311_STATE_EN_PS_MASK) != 0;
  config->enable_config.als_enable = (state_reg & SAPMTK311_STATE_EN_ALS_MASK) != 0;
  config->enable_config.ir_enable = (state_reg & SAPMTK311_STATE_EN_IRS_MASK) != 0;
  config->enable_config.wait_enable = (state_reg & SAPMTK311_STATE_EN_WAIT_MASK) != 0;
  config->enable_config.adc_subtract_offset = (state_reg & SAPMTK311_STATE_EN_ASO_MASK) != 0;
  config->enable_config.interrupt_run_once = (state_reg & SAPMTK311_STATE_EN_IRO_MASK) != 0;
  config->enable_config.interrupt_enable = (int_reg & SAPMTK311_INT_CTRL_MASK) != 0;

  // PS配置
  config->ps_config.gain = (ps_ctrl & SAPMTK311_PS_GAIN_MASK) >> SAPMTK311_PS_GAIN_SHIFT;
  config->ps_config.integration_time = ps_ctrl & SAPMTK311_PS_IT_MASK;
  config->ps_config.persistence = (ps_ctrl & SAPMTK311_PS_PRS_MASK) >> SAPMTK311_PS_PRS_SHIFT;
  config->ps_config.threshold_high = handle->config.ps_config.threshold_high;
  config->ps_config.threshold_low = handle->config.ps_config.threshold_low;
  config->ps_config.auto_calibration = handle->config.ps_config.auto_calibration;

  // ALS配置
  config->als_config.gain = (als_ctrl & SAPMTK311_ALS_GAIN_MASK) >> SAPMTK311_ALS_GAIN_SHIFT;
  config->als_config.integration_time = als_ctrl & SAPMTK311_ALS_IT_MASK;
  config->als_config.persistence = (als_ctrl & SAPMTK311_ALS_PRS_MASK) >> SAPMTK311_ALS_PRS_SHIFT;
  config->als_config.threshold_high = handle->config.als_config.threshold_high;
  config->als_config.threshold_low = handle->config.als_config.threshold_low;
  config->als_config.auto_calibration = handle->config.als_config.auto_calibration;

  // LED配置
  config->led_config.drive_current = (led_ctrl & SAPMTK311_LED_IRDR_MASK) >> SAPMTK311_LED_IRDR_SHIFT;
  config->led_config.duty_cycle = led_ctrl & SAPMTK311_LED_DT_MASK;

  // 中断配置
  config->interrupt_config.ps_mode = int_reg & SAPMTK311_INT_PS_MASK;
  config->interrupt_config.als_mode = (int_reg & SAPMTK311_INT_ALS_MASK) >> SAPMTK311_INT_ALS_SHIFT;
  config->interrupt_config.interrupt_persistence = 0;
  config->interrupt_config.interrupt_output_enable = (int_reg & SAPMTK311_INT_OUI_MASK) != 0;

  // 等待配置
  config->wait_config.wait_time = wait_reg;

  // 电源管理配置
  config->power_config.low_power_mode = handle->config.power_config.low_power_mode;
  config->power_config.power_mode = handle->config.power_config.power_mode;

  return true;
}

/**
 * @brief 统一功能使能函数，根据配置中的使能位，设置状态寄存器中的使能位
 * @param handle 传感器句柄
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_Enable(SAPMTK311_Handle_t *handle)
{
  if (!handle)
  {
    return false;
  }

  // 读取状态寄存器，保留之前其他配置
  uint8_t state_reg;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_STATE_REG, &state_reg))
  {
    return false;
  }

  // 清除所有使能位
  state_reg &= ~(SAPMTK311_STATE_EN_PS_MASK |
                 SAPMTK311_STATE_EN_ALS_MASK |
                 SAPMTK311_STATE_EN_IRS_MASK |
                 SAPMTK311_STATE_EN_WAIT_MASK |
                 SAPMTK311_STATE_EN_ASO_MASK |
                 SAPMTK311_STATE_EN_IRO_MASK);

  // 根据配置设置使能位
  if (handle->config.enable_config.ps_enable)
  {
    state_reg |= SAPMTK311_STATE_EN_PS_MASK;
    handle->ps_enabled = true;
  }
  else
  {
    handle->ps_enabled = false;
  }

  if (handle->config.enable_config.als_enable)
  {
    state_reg |= SAPMTK311_STATE_EN_ALS_MASK;
    handle->als_enabled = true;
  }
  else
  {
    handle->als_enabled = false;
  }

  if (handle->config.enable_config.ir_enable)
  {
    state_reg |= SAPMTK311_STATE_EN_IRS_MASK;
  }

  if (handle->config.enable_config.wait_enable)
  {
    state_reg |= SAPMTK311_STATE_EN_WAIT_MASK;
  }

  if (handle->config.enable_config.adc_subtract_offset)
  {
    state_reg |= SAPMTK311_STATE_EN_ASO_MASK;
  }

  if (handle->config.enable_config.interrupt_run_once)
  {
    state_reg |= SAPMTK311_STATE_EN_IRO_MASK;
  }

  // 写入状态寄存器
  if (!SAPMTK311_WriteByte(handle, SAPMTK311_STATE_REG, state_reg))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteByte in SAPMTK311_Enable failed\r\n");
#endif
    return false;
  }

  return true;
}

/**
 * @brief 设置PS阈值
 * @param handle 传感器句柄
 * @param high 高阈值
 * @param low 低阈值
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_PS_SetThreshold(SAPMTK311_Handle_t *handle, uint16_t high, uint16_t low)
{
  if (!handle)
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_PS_SetThreshold handle or handle->is_initialized failed\r\n");
#endif
    return false;
  }

  // 写入高阈值
  uint8_t high_bytes[2] = {(uint8_t)(high >> 8), (uint8_t)(high & 0xFF)};
  if (!SAPMTK311_WriteBytes(handle, SAPMTK311_THDH1_PS_REG, high_bytes, 2))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteBytes in SAPMTK311_PS_SetThreshold failed\r\n");
#endif
    return false;
  }

  // 写入低阈值
  uint8_t low_bytes[2] = {(uint8_t)(low >> 8), (uint8_t)(low & 0xFF)};
  if (!SAPMTK311_WriteBytes(handle, SAPMTK311_THDL1_PS_REG, low_bytes, 2))
  {
#ifdef SAPMTK311_DEBUG
    printf("SAPMTK311_WriteBytes in SAPMTK311_PS_SetThreshold failed\r\n");
#endif
    return false;
  }

  // 更新内部配置
  handle->config.ps_config.threshold_high = high;
  handle->config.ps_config.threshold_low = low;

  return true;
}

/**
 * @brief 获取PS数据，赋值给data
 * @param handle 传感器句柄
 * @param data PS数据指针
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_PS_GetData(SAPMTK311_Handle_t *handle, uint16_t *data)
{
  if (!handle || !handle->is_initialized || !data)
  {
    return false;
  }

  uint8_t data_high;
  uint8_t data_low;
  if (!SAPMTK311_ReadBytes(handle, SAPMTK311_DATA1_PS_REG, &data_high, 1))
  {
    return false;
  }
  if (!SAPMTK311_ReadBytes(handle, SAPMTK311_DATA2_PS_REG, &data_low, 1))
  {
    return false;
  }
#ifdef SAPMTK311_DEBUG
  printf("SAPMTK311_PS_GetData data_high: 0x%02X, data_low: 0x%02X\r\n", data_high, data_low);
#endif
  *data = (data_high << 8) | data_low;

  return true;
}

/**
 * @brief 获取PS状态
 * @param handle 传感器句柄
 * @return PS状态
 */
SAPMTK311_PS_State_t SAPMTK311_PS_GetState(SAPMTK311_Handle_t *handle, SAPMTK311_PS_State_t *state)
{
  if (!handle || !handle->is_initialized)
  {
    return SAPMTK311_STATE_ERROR;
  }

  uint16_t ps_data;
  if (!SAPMTK311_PS_GetData(handle, &ps_data))
  {
    return SAPMTK311_STATE_ERROR;
  }

  // 根据阈值判断状态
  if (ps_data >= handle->config.ps_config.threshold_high)
  {
    *state = SAPMTK311_STATE_NEAR;
    return SAPMTK311_STATE_NEAR;
  }
  else if (ps_data <= handle->config.ps_config.threshold_low)
  {
    *state = SAPMTK311_STATE_FAR;
    return SAPMTK311_STATE_FAR;
  }
  else
  {
    // 在阈值之间，保持之前的状态
    return *state;
  }
}
/**
 * @brief 设置ALS阈值
 * @param handle 传感器句柄
 * @param high 高阈值
 * @param low 低阈值
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_ALS_SetThreshold(SAPMTK311_Handle_t *handle, uint16_t high, uint16_t low)
{
  if (!handle)
  {
    return false;
  }

  // 写入高阈值
  uint8_t high_bytes[2] = {(uint8_t)(high >> 8), (uint8_t)(high & 0xFF)};
  if (!SAPMTK311_WriteBytes(handle, SAPMTK311_THDH1_ALS_REG, high_bytes, 2))
  {
    return false;
  }

  // 写入低阈值
  uint8_t low_bytes[2] = {(uint8_t)(low >> 8), (uint8_t)(low & 0xFF)};
  if (!SAPMTK311_WriteBytes(handle, SAPMTK311_THDL1_ALS_REG, low_bytes, 2))
  {
    return false;
  }

  // 更新内部配置
  handle->config.als_config.threshold_high = high;
  handle->config.als_config.threshold_low = low;

  return true;
}

/**
 * @brief 获取ALS数据
 * @param handle 传感器句柄
 * @param data ALS数据指针
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_ALS_GetData(SAPMTK311_Handle_t *handle, uint16_t *data)
{
  if (!handle || !handle->is_initialized || !data)
  {
    return false;
  }

  uint8_t als_bytes[2];
  if (!SAPMTK311_ReadBytes(handle, SAPMTK311_DATA1_ALS_REG, als_bytes, 2))
  {
    return false;
  }

  *data = (als_bytes[0] << 8) | als_bytes[1];

  return true;
}

/**
 * @brief 设置LED驱动电流
 * @param handle 传感器句柄
 * @param irdr LED驱动电流设置
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_LED_SetIRDR(SAPMTK311_Handle_t *handle, SAPMTK311_LED_IRDR_t irdr)
{
  if (!handle)
  {
    return false;
  }

  uint8_t led_ctrl;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_LEDCTRL_REG, &led_ctrl))
  {
    return false;
  }

  // 清除并设置驱动电流位
  led_ctrl &= ~SAPMTK311_LED_IRDR_MASK;
  led_ctrl |= (irdr << SAPMTK311_LED_IRDR_SHIFT);

  if (!SAPMTK311_WriteByte(handle, SAPMTK311_LEDCTRL_REG, led_ctrl))
  {
    return false;
  }

  // 更新内部配置
  handle->config.led_config.drive_current = irdr;

  return true;
}

/**
 * @brief 设置LED占空比
 * @param handle 传感器句柄
 * @param duty_cycle 占空比 (0-63)
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_LED_SetDutyCycle(SAPMTK311_Handle_t *handle, uint8_t duty_cycle)
{
  if (!handle || duty_cycle > 63)
  {
    return false;
  }

  uint8_t led_ctrl;
  if (!SAPMTK311_ReadByte(handle, SAPMTK311_LEDCTRL_REG, &led_ctrl))
  {
    return false;
  }

  // 清除并设置占空比位
  led_ctrl &= ~SAPMTK311_LED_DT_MASK;
  led_ctrl |= (duty_cycle & SAPMTK311_LED_DT_MASK);

  if (!SAPMTK311_WriteByte(handle, SAPMTK311_LEDCTRL_REG, led_ctrl))
  {
    return false;
  }

  // 更新内部配置
  handle->config.led_config.duty_cycle = duty_cycle;

  return true;
}

/**
 * @brief 获取标志寄存器
 * @param handle 传感器句柄
 * @param flag 标志值指针
 * @return 成功返回true，失败返回false
 */
bool SAPMTK311_GetFlag(SAPMTK311_Handle_t *handle, uint8_t *flag)
{
  if (!handle || !handle->is_initialized || !flag)
  {
    return false;
  }

  return SAPMTK311_ReadByte(handle, SAPMTK311_FLAG_REG, flag);
}

/**
 * @brief 检查数据是否就绪
 * @param handle 传感器句柄
 * @return 数据就绪返回true，否则返回false
 */
bool SAPMTK311_IsDataReady(SAPMTK311_Handle_t *handle)
{
  if (!handle || !handle->is_initialized)
  {
    return false;
  }

  uint8_t flag;
  if (!SAPMTK311_GetFlag(handle, &flag))
  {
    return false;
  }

  // 检查PS数据就绪标志
  if (handle->ps_enabled && (flag & SAPMTK311_FLG_PSDR_MASK))
  {
    return true;
  }

  // 检查ALS数据就绪标志
  if (handle->als_enabled && (flag & SAPMTK311_FLG_ALSDR_MASK))
  {
    return true;
  }

  return false;
}
