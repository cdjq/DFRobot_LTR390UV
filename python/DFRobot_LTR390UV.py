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
a_gain[5] = {1,3,6,9,18}
a_int[6] = {4.,2.,1.,0.5,0.25,0.25}

eGain1 = 0 #1倍增益
eGain3 = 1 #3倍增益
eGain6 = 2 #6倍增益
eGain9 = 3 #9倍增益
eGain18 =4 #18倍增益

e20bit = 0  #20位数据
e19bit = 16 #19位数据
e18bit = 32 #18位数据
e17bit = 48 #17位数据
e16bit = 64 #16位数据
e13bit =80  #13位数据

e25ms = 0  #采样时间25ms
e50ms = 1  #采样时间50ms
e100ms = 2 #采样时间100ms
e200ms = 3 #采样时间200ms
e500ms = 4 #采样时间500ms
e1000ms =5 #采样时间1000ms
e2000ms = 6 #采样时间2000ms


class DFRobot_LTR390UV():
  def __init__(self ,bus = 0 ,baud = 9600, mode = I2C_MODE):
    self.mode = 0
    self.resolution = 0
    self.gain = 0
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
    rbuf = self._read_reg(0x02,2,0)
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
    self.resolution = (data&0xf0)>>4
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
    self.gain = data
    if self._uart_i2c == I2C_MODE:
      buffer=[data,0]
    else:
      buffer = [data]
    self._write_reg(LTR390UV_HOLDINGREG_ALS_UVS_GAIN,buffer)  
  def read_original_data(self):
    '''
      @brief 获取原始数据
      @return 返回获取得原始数据
    '''
    if self._uart_i2c == I2C_MODE:
      if self.mode == ALSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_ALS_DATA_LOW,4,0)
        data = buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8
        
      elif self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,4,0)
        data = buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8
    else:
      if self.mode == ALSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_ALS_DATA_LOW,2,0)
        data = buffer[0]|buffer[1]<<16
      elif self.mode == UVSMode:
        buffer = self._read_reg(LTR390UV_INPUTREG_UVS_DATA_LOW,2,0)
        data = buffer[0]|buffer[1]<<16
    return data
  
class DFRobot_LTR390UV_I2C(DFRobot_LTR390UV):
  '''!
    @brief An example of an i2c interface module
  '''
  def __init__(self ,bus ,addr):
    self._addr = addr
    DFRobot_LTR390UV.__init__(self,bus,0,I2C_MODE)   
    
  
  def _read_reg(self, reg_addr ,length, state):
    '''!
      @brief read the data from the register
      @param reg_addr register address
      @param length read data
    '''
    if state == 1:
      reg = reg_addr+5
    try:
      rslt = self.i2cbus.read_i2c_block_data(self._addr ,reg , length)
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