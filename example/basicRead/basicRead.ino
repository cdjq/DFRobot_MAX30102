/*!
 * @file blink.ino
 * @brief 输出所有原始的 Red/IR/Green 读数
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
 * @n 本示例支持的主板有ESP8266、FireBeetle-M0、UNO、ESP32、Leonardo 、Mega2560
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
  if (particleSensor.begin() == false) {//使用默认配置
    Serial.println("MAX30102 was not found");
    while (1);
  }
  //传感器配置
  particleSensor.setup(); //使用默认配置
}

void loop()
{
  Serial.print(" R[");
  Serial.print(particleSensor.getRed());
  Serial.print("] IR[");
  Serial.print(particleSensor.getIR());
  Serial.println("]");
}
