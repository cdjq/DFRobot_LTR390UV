# -*- coding: utf-8 -*
'''!
  @file       DFRobot_LTR390UV.py
  @brief       这是LTR390UV传感器基库
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      TangJie(jie.tang@dfrobot.com)
  @version     V1.0
  @date        2021-08-31
  @url         https://github.com/DFRobor/DFRobot_LTR390UV
'''

import serial
import time
import smbus
import os
import math
import RPi.GPIO as GPIO
import math

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

I2C_MODE                  = 0x01
UART_MODE                 = 0x02
DEV_ADDRESS               = 0x1c
ALSMode                   = 0x02
UVSMode                   = 0x0A

#输入寄存器
LTR390UV_INPUTREG_PID                           =0x00   #设备PID
LTR390UV_INPUTREG_VID                           =0x01   #设备的VID,固定为0x3343
LTR390UV_INPUTREG_ADDR                          =0x02   #模块的设备地址
LTR390UV_INPUTREG_BAUDRATE                      =0x03   #串口波特率
LTR390UV_INPUTREG_STOPBIT                       =0x04   #串口校验位和停止位
LTR390UV_INPUTREG_VERSION                       =0x05   #固件版本信息
LTR390UV_INPUTREG_PART_ID                       =0x06   #传感器设备ID
LTR390UV_INPUTREG_ALS_DATA_LOW                  =0x07   #环境光强度低位
LTR390UV_INPUTREG_ALS_DATA_HIGH                 =0x08   #环境光强度高位
LTR390UV_INPUTREG_UVS_DATA_LOW                  =0x09   #紫外线强度低位
LTR390UV_INPUTREG_UVS_DATA_HIGH                 =0x0A   #紫外线强度高位
#保持寄存器
LTR390UV_HOLDINGREG_ALS_UVS_GAIN                =0x06   #增益调节
LTR390UV_HOLDINGREG_INT_CFG                     =0x07   #中断配置
LTR390UV_HOLDINGREG_UVS_ALS_THRES_UP_DATA_LOW   =0x08   #紫外线或环境光阈值上限低位
LTR390UV_HOLDINGREG_UVS_ALS_THRES_UP_DATA_HIGH  =0x09   #紫外线或环境光阈值上限高位
LTR390UV_HOLDINGREG_UVS_ALS_THRES_LOW_DATA_LOW  =0x0A   #紫外线或环境光阈值下限低位
LTR390UV_HOLDINGREG_UVS_ALS_THRES_LOW_DATA_HIGH =0x0B   #紫外线或环境光阈值下限高位
LTR390UV_HOLDINGREG_UVS_ALS_THRES_VAR_DATA      =0x0C   #紫外线或环境光数据变化次数阈值
LTR390UV_HOLDINGREG_ALS_UVS_MEAS_RATE           =0x0D   #数据采集位数和采样时间设置
LTR390UV_HOLDINGREG_MAIN_CTRL                   =0x0E   #传感器模式选择

class DFRobot_LTR390UV():
  def __init__(self ,bus = 0 ,baud = 9600, mode = I2C_MODE):
    self.mode = 0
    if mode == I2C_MODE:
      self.i2cbus = smbus.SMBus(bus)
      self._uart_i2c = I2C_MODE
    else:
      self.master = modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttyAMA0",baudrate=baud, bytesize=8, parity='N', stopbits=1))
      self.master.set_timeout(1.0)
      self._uart_i2c = UART_MODE
  def _detect_device_address(self):
    '''!
      @brief Get sensor address
      @return  Return sensor address
    '''
    rbuf = self._read_reg(0x02,2)
    if self._uart_i2c == I2C_MODE:
      data = rbuf[0] | rbuf[1] << 8
    elif self._uart_i2c == UART_MODE:
      data = rbuf[0]
    return data

  def begin(self):
    '''
      @brief 初始化传感器
    '''
    if self._detect_device_address() != DEV_ADDRESS:
      return False
    return True

  def set_mode(self,mode):
    '''
      @brief 设置模块采集数据模式
      @param mode 采集数据选择
    '''
    self.mode = mode
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [mode]
    self._write_reg(LTR390UV_HOLDINGREG_MAIN_CTRL,buffer) 
  
  def set_ALS_or_UVS_meas_rate(self,data):
    '''
      @brief 设置模块采集数据位数和采集时间，采集时间必须大于采集位数所需时间
      @n --------------------------------------------------------------------------------------------------------
      @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
      @n ---------------------------------------------------------------------------------------------------------
      @n |  Reserved  |        ALS/UVS Resolution            |  Reserved  |   ALS/UVS Measurement Rate           |
      @n ---------------------------------------------------------------------------------------------------------
      @n | ALS/UVS Resolution       |000|20 Bit, Conversion time = 400ms                                         |
      @n |                          |001|19 Bit, Conversion time = 200ms                                         |
      @n |                          |010|18 Bit, Conversion time = 100ms(default)                                |
      @n |                          |011|17 Bit, Conversion time = 50ms                                          |
      @n |                          |100|16 Bit, Conversion time = 25ms                                          |
      @n |                          |110/111|Reserved                                                            |
      @n ---------------------------------------------------------------------------------------------------------
      @n | ALS/UVS Measurement Rate |000|25ms                                                                    |
      @n |                          |001|50ms                                                                    |
      @n |                          |010|100ms (default)                                                         |
      @n |                          |011|200ms                                                                   |
      @n |                          |100|500ms                                                                   |
      @n |                          |101|1000ms                                                                  |
      @n |                          |110/111|2000ms                                                              |
      @n ---------------------------------------------------------------------------------------------------------
      @param data 控制数据
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [data]
    self._write_reg(LTR390UV_HOLDINGREG_ALS_UVS_MEAS_RATE,buffer) 
  def set_ALS_or_UVS_gain(self,data):
    '''
      @brief 设置传感器增益调节
      @n ---------------------------------------------------------------------------------------------------------
      @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
      @n ---------------------------------------------------------------------------------------------------------
      @n |                                    Reserved                    |          ALS/UVS Gain Range          |
      @n ---------------------------------------------------------------------------------------------------------
      @n | ALS/UVS Gain Range       |000|Gain Range: 1                                                           |
      @n |                          |001|Gain Range: 3 (default)                                                 |
      @n |                          |010|Gain Range: 6                                                           |
      @n |                          |011|Gain Range: 9                                                           |
      @n |                          |100|Gain Range: 18                                                          |
      @n |                          |110/111|Reserved                                                            |
      @n ---------------------------------------------------------------------------------------------------------                  
      @param data 控制数据 
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [data]
    self._write_reg(LTR390UV_HOLDINGREG_ALS_UVS_GAIN,buffer) 
  def set_ALD_or_UVS_intcfg(self,data):
    '''
      @brief 环境光中断设置
      @n ---------------------------------------------------------------------------------------------------------
      @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
      @n ---------------------------------------------------------------------------------------------------------
      @n |        Reserved         |         LS_INT_SEL      | LS_VAR_MODE| LS_INT_EN  |    Reserved             |
      @n ---------------------------------------------------------------------------------------------------------
      @n | LS_INT_SEL               |00|Reserved                                                                 |
      @n |                          |01|ALS Channel (Default)                                                    |
      @n |                          |10|Reserved                                                                 |
      @n |                          |11|UVS Channe                                                               |
      @n ---------------------------------------------------------------------------------------------------------
      @n |     LS_VAR_MODE          |0|LS threshold interrupt mode (default)                                     |
      @n |                          |1|LS variation interrupt mode                                               |
      @n --------------------------------------------------------------------------------------------------------- 
      @n |     LS_INT_EN            |0|LS interrupt disabled (default)                                           |
      @n |                          |1|LS interrupt enabled                                                      |
      @n ---------------------------------------------------------------------------------------------------------      
      @param data 控制数据
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [data]
    self._write_reg(LTR390UV_HOLDINGREG_INT_CFG,buffer) 

  def set_UVS_or_ALS_thres_up_data(self,data):
    '''
      @brief 设置中断阈值上限值
      @param data 中断上限阈值，范围0~0x000fffff
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,(data >> 8)&0xff,(data >> 16) & 0xff,0]
    else:
      buffer=[(data&0xffff),(data>>16&0xffff)]
    
    self._write_reg(LTR390UV_HOLDINGREG_UVS_ALS_THRES_UP_DATA_LOW,buffer) 
  
  def set_UVS_or_ALS_thres_low_data(self,data):
    '''
      @brief 设置中断阈值下限值
      @param data 中断下限阈值，范围0~0x000fffff
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,(data >> 8)&0xff,(data >> 16) & 0xff,0]
    else:
      buffer=[(data&0xffff),(data>>16&0xffff)]
    self._write_reg(LTR390UV_HOLDINGREG_UVS_ALS_THRES_LOW_DATA_LOW,buffer)
  
  def read_original_data(self):
    '''
      @brief 获取原始数据
      @return 返回获取得原始数据
    '''
    if self._uart_i2c == I2C_MODE:
      if self.mode == ALSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_ALS_DATA_LOW,4)
        data = buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8
      elif self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,4)
        data = buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8
    else:
      if self.mode == ALSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_ALS_DATA_LOW,2)
        data = buffer[0]|buffer[1]<<16
      elif self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,2)
        data = buffer[0]|buffer[1]<<16
    return data
  
  def read_UVS_transform_data(self):
    '''
      @brief 获取转换后得UVS数据
      @return 返回转换后的数据
    '''
    if self._uart_i2c == I2C_MODE:
      if self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,4)
        data = buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8
    else:
     if self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,2)
        data = buffer[0]|buffer[1]<<16
    returnData = data / 1800
    return returnData

  def set_UVS_or_ALS_thresvar(self,data):
    '''
      @brief 设置环境光或紫外线数据变化次数中断
      @n ---------------------------------------------------------------------------------------------------------
      @n |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
      @n ---------------------------------------------------------------------------------------------------------
      @n |        Reserved                                                |    UVS/ALS Variance Threshold        |
      @n ---------------------------------------------------------------------------------------------------------
      @n | UVS/ALS Variance Threshold   |000|New DATA_x varies by 8 counts compared to previous result           |
      @n |                              |001|New DATA_x varies by 16 counts compared to previous result.         |
      @n |                              |010|New DATA_x varies by 32 counts compared to previous result.         |
      @n |                              |011|New DATA_x varies by 64 counts compared to previous result.         |
      @n |                              |100|New DATA_x varies by 128 counts compared to previous result.        |
      @n |                              |101|New DATA_x varies by 256 counts compared to previous result.        |
      @n |                              |110|New DATA_x varies by 512 counts compared to previous result.        |
      @n |                              |111|New DATA_x varies by 1024 counts compared to previous result.       |
      @n ---------------------------------------------------------------------------------------------------------      
      @param data 发送的数据
    '''
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [data]
    self._write_reg(LTR390UV_HOLDINGREG_UVS_ALS_THRES_VAR_DATA,buffer) 


class DFRobot_LTR390UV_I2C(DFRobot_LTR390UV):
  '''!
    @brief An example of an i2c interface module
  '''
  def __init__(self ,bus ,addr):
    self._addr = addr
    DFRobot_LTR390UV.__init__(self,bus,0,I2C_MODE)   
    
  
  def _read_reg(self, reg_addr ,length):
    '''!
      @brief read the data from the register
      @param reg_addr register address
      @param length read data
    '''
    try:
      rslt = self.i2cbus.read_i2c_block_data(self._addr ,reg_addr , length)
    except:
      rslt = -1
    return rslt

  def _write_reg(self, reg_addr ,data):
    '''!
      @brief write the data from the register
      @param reg_addr register address
      @param data 写入寄存器数据
    '''
    self._reg = reg_addr +5
    try:
      rslt = self.i2cbus.write_i2c_block_data(self._addr ,self._reg , data)
    except:
      rslt = -1
    return rslt        


class DFRobot_LTR390UV_UART(DFRobot_LTR390UV):
  '''!
    @brief An example of an UART interface module
  '''
  def __init__(self ,baud, addr):
    self._baud = baud
    self._addr = addr
    try:
      DFRobot_LTR390UV.__init__(self,0,self._baud,UART_MODE)
    except:
      print ("plese get root!")
   
  
  def _read_reg(self, reg_addr ,length):
    '''!
      @brief Read data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.READ_INPUT_REGISTERS, reg_addr, length))
  
  def _write_reg(self, reg_addr ,data):
    '''!
      @brief write data from the sensor
    '''
    return list(self.master.execute(self._addr, cst.WRITE_MULTIPLE_REGISTERS, reg_addr, output_value=data))