# DFRobot_MAX30102 心率和血氧传感器库
DFRobot_MAX30102是基于MAX30102芯片的心率和血氧浓度传感器库，可以通过被测者的手指得到被测者的心率和血氧浓度，在串口显示实时读数<br>
该库的示例有：显示基础读数，串口绘图器显示心跳，显示心率，显示血氧浓度，显示模块温度<br>
该库提供了所有寄存器的配置方法，用户可以根据需求调用。<br>

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
* 显示心跳 <br>
* 显示血氧浓度 <br>
* 显示模块温度 <br>

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
  /**
   *@brief 构造函数
   */
  void autoColorChange(void);
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

- Data 2020-5-9
- Version V0.1


## Credits

Written by(hangyu.ye@dfrobot.com), 2020. (Welcome to our [website](https://www.dfrobot.com/))





