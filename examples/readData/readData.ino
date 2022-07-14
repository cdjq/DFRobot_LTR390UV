/*!
 *@file  readAlsData.ino
 *@brief 运行本例程可以获取环境光强度，修改模式可以获取紫外线强度
 *@n
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

#define UARTMODE //串口模式
//#define I2CMODE //i2c模式
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
   * @brief 设置模块采集18位数据位数和采集时间100ms
   *  e20bit 20位数据     e25ms   采样时间25ms    e2000ms  采样时间2000ms 
   *  e19bit 19位数据     e50ms   采样时间50ms 
   *  e18bit 18位数据     e100ms  采样时间100ms 
   *  e17bit 17位数据     e200ms  采样时间200ms 
   *  e16bit 16位数据     e500ms  采样时间500ms 
   *  e13bit 13位数据     e1000ms 采样时间1000ms 
   */
  ltr390.setALSOrUVSMeasRate(ltr390.e18bit+ltr390.e100ms);
  /**
   * @brief 设置传感器增益
   *        eGain1 1倍增益
   *        eGain3 3倍增益
   *        eGain6 6倍增益
   *        eGain9 9倍增益
   *        eGain18 18倍增益
   * 
   */
  ltr390.setALSOrUVSGain(ltr390.eGain1);
  ltr390.setMode(ltr390.eALSMode);//设置环境光模式 
  //ltr390.setMode(ltr390.eUVSMode);//设置紫外线模式 
}
void loop()
{
  float als = 0;
  uint32_t data = 0;
  data = ltr390.readOriginalData();//获取环境光或紫外线数据原始数据，根据设置得模式确定
  Serial.print("data:");
  Serial.println(data);
  als = ltr390.readALSTransformData();//获取环境光转换后数据,只能在环境光模式下使用
  Serial.print("ALS:");
  Serial.print(als);
  Serial.println("Lux");
  delay(100);
}
