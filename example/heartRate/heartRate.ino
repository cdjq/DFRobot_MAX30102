/*!
 * @file heartRate.ino
 * @brief 在串口上显示实时心率，心率根据两次心跳的间隔时间求得，第一次显示结果不正确
 * @n 用夹子把传感器固定在手指上，手指按压力度变化会使识别错误，出现异常值
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V0.1
 * @date  2020-04-29
 * @url https://github.com/DFRobot/DFRobot_MAX30102
 */

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
  if (!particleSensor.begin(/*&pWire=*/&Wire, /*i2cSpeed=*/MAX30102_I2C_SPEED)) {//使用默认的I2C端口，I2C速度400kHz
    Serial.println("MAX30102 was not found");
    while (1);
  }
  /*!
   *@brief 传感器配置
   *@param ledBrightness LED灯的亮度，默认值0x1F（6.4mA），取值范围: 0~255（0=关 ，255=50mA）
   *@param sampleAverage 多个样本平均后抽取一次，减少数据吞吐量，默认4个样本平均
   *@param ledMode LED模式选项，默认同时使用红色和红外
   *@param sampleRate 采样速率，默认每秒取400个样本
   *@param pulseWidth 脉冲宽度，脉冲宽度越长，探测范围就越大，默认最大范围，411(µs)
   *@param adcRange ADC量程，默认4096 (nA)，15.63(pA) per LSB
   */
  particleSensor.sensorConfiguration();
  Serial.println("Hold for a while and your heart rate will appear");
}

int32_t heartbeatTime = 0; //最近一次心跳发生的时间
uint32_t BPM; //每分钟心跳
float intervalTime; //与上一次心跳的间隔时间
int32_t IR; //红外读数

void loop()
{
  /*!
   *@brief 获得IR值
   *@return 4字节红外光读数
   */
  IR = particleSensor.getIR();
  /*!
   *@brief 识别一次心跳
   *@param sample 红光或红外光的读数
   *@return true or false
   */
  if(checkForBeat(/*sample=*/IR) == true) {
    //根据心跳间隔时间求出心率
    intervalTime = (millis() - heartbeatTime) / 1000.0;//计算间隔时间
    heartbeatTime = millis();//记录当前时间
    BPM = 60 / intervalTime;
    //打印结果
    Serial.print(F("BPM="));
    Serial.println(BPM);
  }
}
