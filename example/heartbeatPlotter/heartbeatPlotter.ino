/*!
 * @file blink.ino
 * @brief 在Arduino的串行绘图仪上显示用户的心跳，点击工具->“串口绘图器”，确保右下角波特率为115200
 * @n 最好用橡皮筋将传感器固定在手指上，用手按压会改变手指的血液流动，足以影响测量结果
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V0.1
 * @date  2020-03-20
 * @url https://github.com/DFRobot/DFRobot_MAX30102
 */

#include <Wire.h>
#include <DFRobot_MAX30102.h>

DFRobot_MAX30102 particleSensor;

void setup()
{
  //串口初始化
  Serial.begin(115200);
  //传感器初始化
  if (!particleSensor.begin(/*&wirePort=*/Wire, /*i2cSpeed=*/I2C_SPEED_FAST)) {//使用默认的I2C端口，I2C速度400kHz
    Serial.println("MAX30102 was not found");
    while (1);
  }

  //设置合理，使串口绘图器上有清楚的锯齿
  byte ledBrightness = 0x1F;      //取值: 0~255，0=Off ，255=50mA
  byte sampleAverage = 8;         //取值: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;               //取值: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int32_t sampleRate = 100;       //取值: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int32_t pulseWidth = 411;       //取值: 69, 118, 215, 411
  int32_t adcRange = 4096;        //取值: 2048, 4096, 8192, 16384
  //传感器配置
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // //Arduino绘图仪会恼人地自动缩放。为了解决这个问题，预先填充
  // //从传感器得到500个读数，对IR读数进行平均，预先填充绘图仪，使Y刻度接近IR值
  // const byte avgAmount = 64;
  // long baseValue = 0;
  // for (byte x = 0 ; x < avgAmount ; x++)
  // {
  //   baseValue += particleSensor.getIR(); //读取IR值
  // }
  // baseValue /= avgAmount;

  // for (int32_t x = 0 ; x < 500 ; x++)
  //   Serial.println(baseValue);
}

void loop()
{
  //将原始数据发送到串口，打开串口绘图工具查看
  Serial.println(particleSensor.getIR()); 
}
