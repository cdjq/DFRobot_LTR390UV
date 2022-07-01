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
  
  ltr390.setALSOrUVSMeasRate(0x22);//设置模块采集数据位数和采集时间
  ltr390.setALSOrUVSGain(0x01);//设置增益
  ltr390.setALSOrUVSINTCFG(0x10);//不使能中断
  //ltr390.setUVSOrAlsThresUpData(0x3E8);//中断阈值配置
  ltr390.setMode(ltr390.eALSMode);//设置位环境光模式 ，ltr390.eUVSMode（紫外线模式） 
}
void loop()
{
  uint32_t data = 0;
  data = ltr390.readOriginalData();//获取环境光和紫外线原始数据
  //float uvsData = ltr390.readUVSTransformData();//获取紫外线将原始数据转换为0到10范围数据
  Serial.print("ALS:");
  Serial.println(data);
  /*
    Serial.print("UVS:");
    Serial.println(data);
  */
  delay(100);
}
