# -*- coding: utf-8 -*
'''!
  @file  read_data.py
  @brief 运行本例程可以获取环境光强度，修改模式可以获取紫外线强度
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      TangJie(jie.tang@dfrobot.com)
  @version     V1.0
  @date        2021-08-31
  @url         https://github.com/DFRobor/DFRobot_LTR390UV
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time
import RPi.GPIO as GPIO

from DFRobot_LTR390UV import *

ctype=0

ADDRESS = 0x1c 
I2C_1   = 0x01


if ctype==0:
  LTR390UV = DFRobot_LTR390UV_I2C(I2C_1 ,ADDRESS)
else:
  LTR390UV = DFRobot_LTR390UV_UART(9600, ADDRESS)


def setup():
  while (LTR390UV.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor  initialize success!!")
  LTR390UV.set_ALS_or_UVS_meas_rate(e18bit+e100ms)#设置模块采集数据位数和采集时间
  LTR390UV.set_ALS_or_UVS_gain(eGain1)#设置增益
  LTR390UV.set_mode(ALSMode)#设置位环境光模式 ，UVSMode（紫外线模式） 
  
def loop():


  data = LTR390UV.read_original_data()#获取环境光和紫外线原始数据
  #uvsData = ltr390.read_UVS_transform_data()#获取紫外线将原始数据转换为0到10范围数据
  print("-----------------------\r\n")
  print("ALS:" + str(data) + "\r\n")
  print("-----------------------\r\n")
  time.sleep(1)

if __name__ == "__main__":
  setup()
  while True:
    loop()