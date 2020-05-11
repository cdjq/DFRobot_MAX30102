/*!
 * @file SPO2.ino
 * @brief 在串口上显示实时心率和SPO2水平，正常人的血氧浓度范围在95~100之间
 * @n 最好用橡皮筋把传感器固定在手指上，手指按压力度的变化会导致读数不稳定，每次力度变化，会在一段时间后恢复正常
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
#include <spo2_algorithm.h>

DFRobot_MAX30102 particleSensor;

//Arduino Uno使用16位缓冲区存放数据
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100];
uint16_t redBuffer[100];
#else
uint32_t irBuffer[100];
uint32_t redBuffer[100];
#endif
int32_t bufferLength = 100;//100的缓冲区长度存储4秒样本
int32_t spo2; //血氧浓度
int8_t validSPO2; //显示SPO2计算是否有效的标志
int32_t heartRate; //心率
int8_t heartRateValid; //显示心率计算是否有效的标志

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
    Serial.println(F("MAX30102 was not found"));
    while (1);
  }

  /*!
   *@brief 传感器配置
   *@param ledBrightness LED灯的亮度，默认值0x1F（6.4mA），取值范围: 0~255（0=Off ，255=50mA）
   *@param sampleAverage 多个样本平均后抽取一次，减少数据吞吐量，默认4个样本平均
   *@param ledMode LED模式选项，默认同时使用红色传感器和红外传感器
   *@param sampleRate 采样速率，默认每秒取400个样本
   *@param pulseWidth 脉冲宽度，脉冲宽度越长，探测范围就越大，默认最大范围，411(µs)
   *@param adcRange ADC量程，默认4096 (nA)，15.63(pA) per LSB
   */
  particleSensor.sensorConfiguration(/*ledBrightness=*/50, /*sampleAverage=*/MAX30102_SAMPLEAVG_4, \
                        /*ledMode=*/MAX30102_MODE_RED_IR, /*sampleRate=*/MAX30102_SAMPLERATE_100, \
                        /*pulseWidth=*/MAX30102_PULSEWIDTH_411, /*adcRange=*/MAX30102_ADCRANGE_16384);

  //采集100个样本
  sampleCollection();
  /**
   *@brief 计算前100个样本的心率和血氧饱和度
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
}

void loop()
{
  //更新样本
  sampleUpdate();
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


void sampleCollection()
{
//打开串口后等待一段时间，确保手指已经固定好，才能开始采样
  Serial.println("Please put your finger on the sensor and wait a moment");
  delay(5000);

  //读取100个样本
  for (uint8_t i = 0 ; i < 100 ; i++) {
    while (particleSensor.available() == 0) {//计算缓冲区中可用样本数，如果读出数据则跳出循环
      particleSensor.foundData(1); //读取数据，存放在缓冲区
    }
    redBuffer[i] = particleSensor.getFIFORed();
    irBuffer[i] = particleSensor.getFIFOIR();
    particleSensor.nextSample();
    // Serial.print(F("red="));
    // Serial.print(redBuffer[i], DEC);
    // Serial.print(F(", ir="));
    // Serial.println(irBuffer[i], DEC);
  }
}

void sampleUpdate()
{
  //将75组样本前移
  for (uint8_t i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }
  //采集25组新样本，放在末尾
  for (uint8_t i = 75; i < 100; i++) {
    while (particleSensor.available() == 0) {
      particleSensor.foundData(1); 
    }
    redBuffer[i] = particleSensor.getFIFORed();
    irBuffer[i] = particleSensor.getFIFOIR();
    particleSensor.nextSample();
  }
}