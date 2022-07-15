/*!
 * @file DFRobot_LTR390UV.h
 * @brief 这是LTR390UV的方法说明文件
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [TangJie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2022-05-17
 * @url https://github.com/DFRobor/DFRobot_LTR390UV
 */
#ifndef DFROBOT_LTR390UV_H
#define DFROBOT_LTR390UV_H

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_RTU.h"
#include "String.h"
#if (defined ARDUINO_AVR_UNO) && (defined ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

//#define ENABLE_DBG ///< 打开这个宏, 可以看到程序的详细运行过程
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

class DFRobot_LTR390UV:public DFRobot_RTU{
public:
  #define LTR390UV_DEVICE_ADDR                            0X1C
  #define LTR390UV_DEVICE_PID                             0X021C
  //输入寄存器
  #define LTR390UV_INPUTREG_PID                           0x00   ///< 设备PID
  #define LTR390UV_INPUTREG_VID                           0x01   ///<设备的VID,固定为0x3343
  #define LTR390UV_INPUTREG_ADDR                          0x02   ///<模块的设备地址
  #define LTR390UV_INPUTREG_BAUDRATE                      0x03   ///<串口波特率
  #define LTR390UV_INPUTREG_STOPBIT                       0x04   ///<串口校验位和停止位
  #define LTR390UV_INPUTREG_VERSION                       0x05   ///<固件版本信息
  #define LTR390UV_INPUTREG_PART_ID                       0x06   ///<传感器设备ID
  #define LTR390UV_INPUTREG_ALS_DATA_LOW                  0x07   ///<环境光强度低位
  #define LTR390UV_INPUTREG_ALS_DATA_HIGH                 0x08   ///<环境光强度高位
  #define LTR390UV_INPUTREG_UVS_DATA_LOW                  0x09   ///<紫外线强度低位
  #define LTR390UV_INPUTREG_UVS_DATA_HIGH                 0x0A   ///<紫外线强度高位
  //保持寄存器
  #define LTR390UV_HOLDINGREG_ALS_UVS_GAIN                0x06   ///<增益调节
  #define LTR390UV_HOLDINGREG_INT_CFG                     0x07   ///<中断配置
  #define LTR390UV_HOLDINGREG_UVS_ALS_THRES_UP_DATA_LOW   0x08   ///<紫外线或环境光阈值上限低位
  #define LTR390UV_HOLDINGREG_UVS_ALS_THRES_UP_DATA_HIGH  0x09   ///<紫外线或环境光阈值上限高位
  #define LTR390UV_HOLDINGREG_UVS_ALS_THRES_LOW_DATA_LOW  0x0A   ///<紫外线或环境光阈值下限低位
  #define LTR390UV_HOLDINGREG_UVS_ALS_THRES_LOW_DATA_HIGH 0x0B   ///<紫外线或环境光阈值下限高位
  #define LTR390UV_HOLDINGREG_UVS_ALS_THRES_VAR_DATA      0x0C   ///<紫外线或环境光数据变化次数阈值
  #define LTR390UV_HOLDINGREG_ALS_UVS_MEAS_RATE           0x0D   ///<数据采集位数和采样时间设置
  #define LTR390UV_HOLDINGREG_MAIN_CTRL                   0x0E   ///<传感器模式选择



  
  



  /**
   * @enum enum 
   * @brief 模块采集数据模式设置
   */
    typedef enum{
      eALSMode = 0x02,
      eUVSMode = 0x0A
    }eModel_t;

  /**
   * @enum enum 
   * @brief 设置模块增益
   */
    typedef enum{
      eGain1 = 0,
      eGain3,
      eGain6,
      eGain9,
      eGain18

    }eGainRange;
  /**
   * @enum enum 
   * @brief 设置数据位数
   */
    typedef enum{
      e20bit = 0,
      e19bit = 16,
      e18bit = 32,
      e17bit = 48,
      e16bit = 64,
      e13bit =80
    }eResolution;

  /**
   * @enum enum 
   * @brief 设置采样时间
   */
    typedef enum{
      e25ms = 0,
      e50ms = 1,
      e100ms = 2,
      e200ms = 3,
      e500ms = 4,
      e1000ms =5,
      e2000ms = 6
    }eMeasurementRate;

  /**
   * @fn DFRobot_LTR390UV
   * @brief DFRobot_LTR390UV constructor
   * @param pWire I2C pointer to the TowWire stream, which requires calling begin in the demo to init Arduino I2C config.
   * @param addr  I2C communication address of SEN0540 device
   */
  DFRobot_LTR390UV(uint8_t addr, TwoWire *pWire = &Wire);

  /**
   * @fn DFRobot_LTR390UV
   * @brief DFRobot_LTR390UV constructor
   * @param addr: The device address of the communication between the host computer and SEN0540 slave device
   * @n     SEN0540_DEVICE_ADDR or 28(0X1C): Default address of SEN0540 device, if users do not change the device address, it's default to 28.
   * @param s   : The serial port pointer to the Stream, which requires calling begin in the demo to init communication serial port config of Arduino main controller, in line with that of SEN0540 device slave.
   * @n SEN0540 serial port config: 9600 baud rate, 8-bit data bit, no check bit, 1 stop bit, the parameters can't be changed.
   */
  DFRobot_LTR390UV(uint8_t addr, Stream *s);
  ~DFRobot_LTR390UV(){};

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
   * @n |                          |101|13 Bit, Conversion time = 12.5ms                                          |
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
   * @param bit 设置数据位数
   * @param time 设置采样时间
   * @return None
   */
  void setALSOrUVSMeasRate(eResolution bit,eMeasurementRate time);

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
  void setALSOrUVSGain(eGainRange data);

  /**
   * @fn readData
   * @brief 获取原始数据
   * @return 返回获取得原始数据
   */
  uint32_t readOriginalData(void);

  /**
   * @fn readALSTransformData
   * @brief 获取转换后得ALS数据
   * @return 返回转换后的数据
   */
  float readALSTransformData(void);
protected:
  bool detectDeviceAddress(uint8_t addr);
  uint8_t  readReg(uint16_t reg, void *pBuf, uint8_t size,uint8_t state);
  uint8_t writeReg(uint8_t reg, void *pBuf, size_t size);
  TwoWire   *_pWire = NULL;
  Stream    *_s = NULL;
  uint8_t   _addr;
  uint8_t _mode;
  
  uint8_t a_gain[5] = {1,3,6,9,18};
  double a_int[6] = {4.,2.,1.,0.5,0.25,0.03125};
  uint8_t gain = 0,resolution = 0;
};


#endif
