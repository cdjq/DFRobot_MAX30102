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
/*
  Optical SP02 Detection (SPK Algorithm) using the DFRobot_MAX30102 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30102_Breakout

  This demo shows heart rate and SPO2 levels.

  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.

  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun DFRobot_MAX30102 library and to compile under Arduino 1.6.11
  Please see license file for more info.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The DFRobot_MAX30102 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include <DFRobot_MAX30102.h>
#include <spo2_algorithm.h>

DFRobot_MAX30102 particleSensor;

void setup()
{
  //串口初始化
  Serial.begin(115200); 
  //传感器初始化
  if (!particleSensor.begin(/*&wirePort=*/Wire, /*i2cSpeed=*/I2C_SPEED_FAST)) {//使用默认的I2C端口，I2C速度400kHz
    Serial.println(F("MAX30102 was not found"));
    while (1);
  }

  byte ledBrightness = 60;  //取值: 0~255，0=Off ，255=50mA
  byte sampleAverage = 4;   //取值: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //取值: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;    //取值: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int32_t pulseWidth = 411; //取值: 69, 118, 215, 411
  int32_t adcRange = 4096;  //取值: 2048, 4096, 8192, 16384

  //传感器配置
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

//Arduino Uno使用16位缓冲区存放数据
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100];
uint16_t redBuffer[100];
#else
uint32_t irBuffer[100];
uint32_t redBuffer[100];
#endif

int32_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t heartRateValid; //indicator to show if the heart rate calculation is valid

void loop()
{
  //读取前100个样本，并确定信号范围
  for (byte i = 0 ; i < 100 ; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check(); //检查传感器是否有新的数据
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //下一个样本

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &heartRateValid);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //在计算心率前取25组样本
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) {
        particleSensor.check(); //Check the sensor for new data
      }
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //下一个样本
    }

    //收集25个新样本后，计算HeartRate和SPO2

    /**
     *@brief 计算心率和血氧饱和度
     *@param *pun_ir_buffer            [in]红外传感器数据缓冲区
     *@param n_ir_buffer_length        [in]红外传感器数据缓冲区长度
     *@param *pun_red_buffer           [in]红色传感器数据缓冲区
     *@param *pn_spo2                  [out]计算的SpO2值
     *@param *pch_spo2_valid           [out]如果计算的SpO2值是有效的，值为1
     *@param *pn_heart_rate            [out]计算的心率值
     *@param *pch_hr_valid             [out]如果计算出的心率值是有效的，值为1
    */
    maxim_heart_rate_and_oxygen_saturation(/**pun_ir_buffer=*/irBuffer, /*n_ir_buffer_length=*/bufferLength, /**pun_red_buffer=*/redBuffer, \
                        /**pn_spo2=*/&spo2, /**pch_spo2_valid=*/&validSPO2, /**pn_heart_rate=*/&heartRate, /**pch_hr_valid=*/&heartRateValid);
    Serial.print(F("HeartRate="));
    Serial.print(heartRate, DEC);
    Serial.print(F(", heartRateValid="));
    Serial.print(heartRateValid, DEC);
    Serial.print(F("; SPO2="));
    Serial.print(spo2, DEC);
    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
  }
}

