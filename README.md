# DFRobot_MAX30102 心率和血氧传感器库
DFRobot_MAX30102是基于MAX30102芯片的心率血氧传感器库。
MAX30102是一个集成的脉搏血氧仪和心率检测仪生物的传感器，MAX30102采用PPG光电容积脉搏波描记法(PhotoPlethysmoGraphy)测量心率。
传感器小巧易佩戴，可佩戴于手指和手腕等处进行数据采集。内部集成了18位ADC采集器，并通过I2C输出数据，兼容大多数主控。<br>
该库的示例有：串口上显示实时基础读数，串口绘图器显示心跳，计算心率、血氧浓度<br>
该库提供了所有寄存器的配置方法，可根据需要公开调用，自由控制MAX30102。<br>

## Product Link （链接到英文商城）
    SKU：产品名称
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

* 心跳识别和心率显示 <br>
* 测血氧浓度 <br>


## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /*!
   *@brief 传感器初始化
   *@param pWire IIC bus pointer object and construction device, can both pass or not pass parameters (Wire in default)
   *@param i2cAddr Chip IIC address (0x57 in default)
   *@return true or false
   */
  bool begin(TwoWire *pWire = &Wire, uint8_t i2cAddr = MAX30102_IIC_ADDRESS);

  /*!
   *@brief 传感器配置，使用给出的宏定义进行配置
   *@param ledBrightness LED灯的亮度，默认值0x1F（6.4mA），取值范围: 0~255（0=Off ，255=50mA）
   *@param sampleAverage 多个样本平均后抽取一次，减少数据吞吐量，默认4个样本平均
   *@param ledMode LED灯的模式，默认同时使用红光和红外光
   *@param sampleRate 采样速率，默认每秒取400个样本
   *@param pulseWidth 脉冲宽度，脉冲宽度越长，探测范围就越大，默认最大范围
   *@param adcRange ADC量程，默认4096 (nA)，15.63(pA) per LSB
   */
  void sensorConfiguration(uint8_t ledBrightness = 0x1F, uint8_t sampleAverage = SAMPLEAVG_4, \
                           uint8_t ledMode = MODE_MULTILED, uint8_t sampleRate = SAMPLERATE_400, \
                           uint8_t pulseWidth = PULSEWIDTH_411, uint8_t adcRange = ADCRANGE_4096);

  /*!
   *@brief 获得red值
   *@return 红光读数
   */
  uint32_t getRed(void);

  /*!
   *@brief 获得IR值
   *@return 红外光读数
   */
  uint32_t getIR(void);

  /*!
   *@brief 获取模块温度，单位是摄氏度
   *@return 浮点型温度值
   */
  float readTemperatureC();

  /*!
   *@brief 获取模块温度，单位是华氏度
   *@return 浮点型温度值
   */
  float readTemperatureF();

  /*!
   *@brief 计算心率和血氧饱和度
   *@param *SPO2                  [out]计算的SpO2值
   *@param *SPO2Valid             [out]如果计算的SpO2值是有效的，值为1
   *@param *heartRate             [out]计算的心率值
   *@param *heartRateValid        [out]如果计算出的心率值是有效的，值为1
   */
  void heartrateAndOxygenSaturation(int32_t* SPO2,int8_t* SPO2Valid,int32_t* heartRate,int8_t* heartRateValid);

```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino uno        |      √       |              |             | 
Mega2560        |      √       |              |             | 
Leonardo        |      √       |              |             | 
ESP32        |      √       |              |             | 
ESP8266        |      √       |              |             | 
FireBeetle-M0        |      √       |              |             | 


## History

- Data 2020-6-1
- Version V0.1


## Credits

Written by(hangyu.ye@dfrobot.com), 2020. (Welcome to our [website](https://www.dfrobot.com/))

