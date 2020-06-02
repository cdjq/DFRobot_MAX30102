/*!
 * @file heartbeatPlotter.ino
 * @brief 在Arduino的串行绘图仪上显示用户的心跳，点击工具->“串口绘图器”，确保右下角波特率为115200
 * @n 最好用橡皮筋将传感器固定在手指上，用手按压会改变手指的血液流动，足以影响测量结果
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V0.1
 * @date  2020-05-29
 * @url https://github.com/DFRobot/DFRobot_MAX30102
 */

#include <DFRobot_MAX30102.h>

DFRobot_MAX30102 particleSensor;

/*
传感器配置中使用的宏定义选项
sampleAverage: SAMPLEAVG_1 SAMPLEAVG_2 SAMPLEAVG_4 
               SAMPLEAVG_8 SAMPLEAVG_16 SAMPLEAVG_32
ledMode:       MODE_REDONLY  MODE_RED_IR  MODE_MULTILED
sampleRate:    PULSEWIDTH_69 PULSEWIDTH_118 PULSEWIDTH_215 PULSEWIDTH_411
pulseWidth:    SAMPLERATE_50 SAMPLERATE_100 SAMPLERATE_200 SAMPLERATE_400
               SAMPLERATE_800 SAMPLERATE_1000 SAMPLERATE_1600 SAMPLERATE_3200
adcRange:      ADCRANGE_2048 ADCRANGE_4096 ADCRANGE_8192 ADCRANGE_16384
*/
void setup()
{
  //串口初始化
  Serial.begin(115200);
  /*!
   *@brief 传感器初始化
   *@param pWire IIC bus pointer object and construction device, can both pass or not pass parameters (Wire in default)
   *@param i2cAddr Chip IIC address (0x57 in default)
   *@return true or false
   */
  while (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
    delay(1);
  }

  //设置合理，使串口绘图器上有清楚的锯齿
  /*!
   *@brief 传感器配置，使用给出的宏定义进行配置
   *@param ledBrightness LED灯的亮度，默认值0x1F（6.4mA），取值范围: 0~255（0=Off ，255=50mA）
   *@param sampleAverage 多个样本平均后抽取一次，减少数据吞吐量，默认4个样本平均
   *@param ledMode LED灯的模式，默认同时使用红光和红外光
   *@param sampleRate 采样速率，默认每秒取400个样本
   *@param pulseWidth 脉冲宽度，脉冲宽度越长，探测范围就越大，默认最大范围
   *@param adcRange ADC量程，默认4096 (nA)，15.63(pA) per LSB
   */
  particleSensor.sensorConfiguration(/*ledBrightness=*/60, /*sampleAverage=*/SAMPLEAVG_8, \
                                  /*ledMode=*/MODE_RED_IR, /*sampleRate=*/SAMPLERATE_400, \
                                  /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_16384);
}

void loop()
{
  //将原始数据发送到串口，打开串口绘图工具查看
  /*!
   *@brief 获得IR值
   *@return 红外光读数
   */
  Serial.println(particleSensor.getIR()); 
}
