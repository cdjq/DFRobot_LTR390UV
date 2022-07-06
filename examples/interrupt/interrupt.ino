/*!
 *@file  interrupt.ino
 *@brief 运行本例程，通过使能中断和设置中断阈值，触发中断打印信息
 * @n connected table
 * ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------
 * 
 * @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [TangJie](jie.tang@dfrobot.com)
 * @version     V1.0
 * @date        2021-08-31
 * @url         https://github.com/DFRobor/DFRobot_LTR390UV
 */
#include "DFRobot_LTR390UV.h"

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif
//#define UARTMODE //串口模式
#define I2CMODE //i2c模式
#if defined UARTMODE
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_LTR390UV ltr390(/*addr =*/LTR390UV_DEVICE_ADDR, /*s =*/&mySerial);
#else
  DFRobot_LTR390UV ltr390(/*addr =*/LTR390UV_DEVICE_ADDR, /*s =*/&Serial1);
#endif
#endif
#if defined I2CMODE
DFRobot_LTR390UV ltr390(/*addr = */LTR390UV_DEVICE_ADDR, /*pWire = */&Wire);
#endif
uint8_t state = 0;
void setup()
{
  
#if defined UARTMODE
  //Init MCU communication serial port
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  mySerial.begin(9600);
#elif defined(ESP32)
  Serial1.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  Serial1.begin(9600);
#endif
#endif
  Serial.begin(115200);
  
  while(ltr390.begin() != 0){
    Serial.println(" Sensor initialize failed!!");
    delay(1000);
  }
  Serial.println(" Sensor  initialize success!!");
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
   * @return None
   */
  ltr390.setALSOrUVSMeasRate(0x22);//设置模块采集18位数据位数和采集时间100ms
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
  ltr390.setALSOrUVSGain(0x01);//设置增益
  ltr390.setALSOrUVSINTCFG(0x14);//使能中断
  ltr390.setUVSOrAlsThresUpData(0x4E20);//中断阈值配置,范围0x00~0x3ffff,范围最高值根据设为位数
  ltr390.setMode(ltr390.eALSMode);//设置位环境光模式 ，ltr390.eUVSMode（紫外线模式） 
  attachInterrupt(digitalPinToInterrupt(2), interrupt, CHANGE);
}
void loop()
{
  delay(100);
}
void interrupt(void)
{
  Serial.println("interrupt!!");
}
