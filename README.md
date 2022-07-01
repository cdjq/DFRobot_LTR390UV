DFRobot_LTR390UV
===========================

* [中文版](./README_CN.md)

SEN0540是一个可以作为环境光获取或紫外线获取得传感器，能过使用用户库方便快捷得使用传感器。

![产品效果图片](../../resources/images/SEN0540.png)
  
## Product Link (https://www.dfrobot.com)
    SKU: SEN0540

## Table of Contents

  * [Summary](#summary)
  * [Installation](#installation)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Summary

This multifunctional environmental sensor library can help obtain information like temperature, humidity, pressure, UV intensity, natural sunlight intensity and altitude.
The module also offers Gravity and breakout version for easy use.


## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /**
   * @fn begin
   * @brief Init SEN0540 device
   * @return Return value init status
   * @retval 0  Succeed
   * @retval -1 Failed
   */
  int8_t begin(void);

  /**
   * @fn setMode
   * @brief 设置模块采集数据模式
   * @param mode 采集数据选择
   * @return NONE
   */
  void setMode(eModel_t mode);

  /**
   * @fn setALSOrUVSMeasRate
   * @brief 设置模块采集数据位数和采集时间，采集时间必须大于采集位数所需时间
   * @n --------------------------------------------------------------------------------------------------------
   * @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |  Reserved  |        ALS/UVS Resolution            |  Reserved  |   ALS/UVS Measurement Rate           |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n | ALS/UVS Resolution       |000|20 Bit, Conversion time = 400ms                                         |
   * @n |                          |001|19 Bit, Conversion time = 200ms                                         |
   * @n |                          |010|18 Bit, Conversion time = 100ms(default)                                |
   * @n |                          |011|17 Bit, Conversion time = 50ms                                          |
   * @n |                          |100|16 Bit, Conversion time = 25ms                                          |
   * @n |                          |110/111|Reserved                                                            |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n | ALS/UVS Measurement Rate |000|25ms                                                                    |
   * @n |                          |001|50ms                                                                    |
   * @n |                          |010|100ms (default)                                                         |
   * @n |                          |011|200ms                                                                   |
   * @n |                          |100|500ms                                                                   |
   * @n |                          |101|1000ms                                                                  |
   * @n |                          |110/111|2000ms                                                              |
   * @n ---------------------------------------------------------------------------------------------------------
   * @param data 控制数据
   * @return None
   */
  void setALSOrUVSMeasRate(uint8_t data);

  /**
   * @fn setALSOrUVSGain
   * @brief 设置传感器增益调节
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |                                    Reserved                    |          ALS/UVS Gain Range          |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n | ALS/UVS Gain Range       |000|Gain Range: 1                                                           |
   * @n |                          |001|Gain Range: 3 (default)                                                 |
   * @n |                          |010|Gain Range: 6                                                           |
   * @n |                          |011|Gain Range: 9                                                           |
   * @n |                          |100|Gain Range: 18                                                          |
   * @n |                          |110/111|Reserved                                                            |
   * @n ---------------------------------------------------------------------------------------------------------                  
   * @param data 控制数据 
   * @return None
   */
  void setALSOrUVSGain(uint8_t data);

  /**
   * @fn setALSOrUVSINTCFG
   * @brief 环境光中断设置
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |        Reserved         |         LS_INT_SEL      | LS_VAR_MODE| LS_INT_EN  |    Reserved             |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n | LS_INT_SEL               |00|Reserved                                                                 |
   * @n |                          |01|ALS Channel (Default)                                                    |
   * @n |                          |10|Reserved                                                                 |
   * @n |                          |11|UVS Channe                                                               |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |     LS_VAR_MODE          |0|LS threshold interrupt mode (default)                                     |
   * @n |                          |1|LS variation interrupt mode                                               |
   * @n --------------------------------------------------------------------------------------------------------- 
   * @n |     LS_INT_EN            |0|LS interrupt disabled (default)                                           |
   * @n |                          |1|LS interrupt enabled                                                      |
   * @n ---------------------------------------------------------------------------------------------------------      
   * @param data 控制数据
   * @return None
   */
  void setALSOrUVSINTCFG(uint8_t data);

  /**
   * @fn setUVSOrAlsThresUpData
   * @brief 设置中断阈值上限值
   * @param data 中断上限阈值，范围0~0x000fffff
   */
  uint8_t setUVSOrAlsThresUpData(uint32_t data);

   /**
   * @fn setUVSOrAlsThresUpData
   * @brief 设置中断阈值下限值
   * @param data 中断下限阈值，范围0~0x000fffff
   */
  uint8_t setUVSOrAlsThresLowData(uint32_t data);

  /**
   * @fn readData
   * @brief 获取原始数据
   * @return 返回获取得原始数据
   */
  uint32_t readOriginalData(void);

  /**
   * @fn readUVSTransformData
   * @brief 获取转换后得UVS数据
   * @return 返回转换后的数据
   */
  float readUVSTransformData(void);
  /**
   * @fn 
   * @brief 设置环境光或紫外线数据变化次数中断
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n |        Reserved                                                |    UVS/ALS Variance Threshold        |
   * @n ---------------------------------------------------------------------------------------------------------
   * @n | UVS/ALS Variance Threshold   |000|New DATA_x varies by 8 counts compared to previous result           |
   * @n |                              |001|New DATA_x varies by 16 counts compared to previous result.         |
   * @n |                              |010|New DATA_x varies by 32 counts compared to previous result.         |
   * @n |                              |011|New DATA_x varies by 64 counts compared to previous result.         |
   * @n |                              |100|New DATA_x varies by 128 counts compared to previous result.        |
   * @n |                              |101|New DATA_x varies by 256 counts compared to previous result.        |
   * @n |                              |110|New DATA_x varies by 512 counts compared to previous result.        |
   * @n |                              |111|New DATA_x varies by 1024 counts compared to previous result.       |
   * @n ---------------------------------------------------------------------------------------------------------      
   * @param data 发送的数据
   */
  void setUvsOrAlsThresVar(uint8_t data);
```

## Compatibility

MCU                | SoftwareSerial | HardwareSerial |      IIC      |
------------------ | :----------: | :----------: | :----------: | 
Arduino Uno        |      √       |      X       |      √       |
Mega2560           |      √       |      √       |      √       |
Leonardo           |      √       |      √       |      √       |
ESP32              |      X       |      √       |      √       |
ESP8266            |      √       |      X       |      √       |
micro:bit          |      X       |      X       |      √       |
FireBeetle M0      |      X       |      √       |      √       |
Raspberry Pi       |      X       |      √       |      √       |

## History

- 2022-06-30 - Version 1.0.0 released.

## Credits

Written by TangJie(jie.tang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))
