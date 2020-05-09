/*!
 * @file heartbeatPlotter.ino
 * @brief 在Arduino的串行绘图仪上显示用户的心跳，点击工具->“串口绘图器”，确保右下角波特率为115200
 * @n 最好用橡皮筋将传感器固定在手指上，用手按压会改变手指的血液流动，足以影响测量结果
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V0.1
 * @date  2020-04-29
 * @url https://github.com/DFRobot/DFRobot_MAX30102
 */

#include <Wire.h>
#include <DFRobot_MAX30102.h>

DFRobot_MAX30102 particleSensor;

/*
传感器配置中使用的宏定义选项
sampleAverage: MAX30102_SAMPLEAVG_1 MAX30102_SAMPLEAVG_2 MAX30102_SAMPLEAVG_4 
               MAX30102_SAMPLEAVG_8 MAX30102_SAMPLEAVG_16 MAX30102_SAMPLEAVG_32
ledMode:       MAX30102_MODE_REDONLY  MAX30102_MODE_RED_IR  MAX30102_MODE_MULTILED
sampleRate:    MAX30102_PULSEWIDTH_69 MAX30102_PULSEWIDTH_118 MAX30102_PULSEWIDTH_215 MAX30102_PULSEWIDTH_411
pulseWidth:    MAX30102_SAMPLERATE_50 MAX30102_SAMPLERATE_100 MAX30102_SAMPLERATE_200 MAX30102_SAMPLERATE_400
               MAX30102_SAMPLERATE_800 MAX30102_SAMPLERATE_1000 MAX30102_SAMPLERATE_1600 MAX30102_SAMPLERATE_3200
adcRange:      MAX30102_ADCRANGE_2048 MAX30102_ADCRANGE_4096 MAX30102_ADCRANGE_8192 MAX30102_ADCRANGE_16384
*/
void setup()
{
  //串口初始化
  Serial.begin(115200);
  /*!
   *@brief 传感器初始化
   *@param pWire IIC bus pointer object and construction device, can both pass or not pass parameters (Wire in default)
   *@param i2cSpeed I2C speed (100000 in default)
   *@param i2cAddr Chip IIC address (0x57 in default)
   *@return true or false
   */
  if (!particleSensor.begin(/*&pWire=*/&Wire, /*i2cSpeed=*/I2C_SPEED_FAST)) {//使用默认的I2C端口，I2C速度400kHz
    Serial.println("MAX30102 was not found");
    while (1);
  }

  //设置合理，使串口绘图器上有清楚的锯齿
  /*!
   *@brief 传感器配置
   *@param ledBrightness LED灯的亮度，默认值0x1F（6.4mA），取值范围: 0~255（0=Off ，255=50mA）
   *@param sampleAverage 多个样本平均后抽取一次，减少数据吞吐量，默认4个样本平均
   *@param ledMode LED模式选项，默认同时使用红色传感器和红外传感器
   *@param sampleRate 采样速率，默认每秒取400个样本
   *@param pulseWidth 脉冲宽度，脉冲宽度越长，探测范围就越大，默认最大范围，411(µs)
   *@param adcRange ADC量程，默认4096 (nA)，15.63(pA) per LSB
   */
  particleSensor.sensorConfiguration(/*ledBrightness=*/60, /*sampleAverage=*/MAX30102_SAMPLEAVG_8, \
                                  /*ledMode=*/MAX30102_MODE_RED_IR, /*sampleRate=*/MAX30102_SAMPLERATE_400, \
                                  /*pulseWidth=*/MAX30102_PULSEWIDTH_411, /*adcRange=*/MAX30102_ADCRANGE_16384);
}

void loop()
{
  while (particleSensor.available() == 0) {//计算缓冲区中可用样本数，如果读出数据则跳出循环
    particleSensor.foundData(1); //读取数据
  }
  while(1) {
    //将原始数据发送到串口，打开串口绘图工具查看
    Serial.println(particleSensor.getIR()); 
  }
}
