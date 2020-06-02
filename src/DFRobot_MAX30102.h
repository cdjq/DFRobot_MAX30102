/*!
 * @file DFRobot_MAX30102.h
 * @brief Define the basic structure of class DFRobot_MAX30102
 * @n This is a library used to drive heart rate and oximeter sensor
 * @n 可以自由控制传感器，采集红光和红外光读数，包含了心率和血氧饱和度的算法
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V1.0
 * @date  2020-05-30
 * @https://github.com/DFRobot/DFRobot_MAX30102
 */
#ifndef _DFROBOT_MAX30102_H
#define _DFROBOT_MAX30102_H

#include <Arduino.h>
#include <Wire.h>
#include <SPO2/algorithm.h>
#include <PBA/heartRate.h>

//Open this macro to see the detailed running process of the program
// #define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#define MAX30102_IIC_ADDRESS  0x57 //I2C Address

//Status Registers
#define MAX30102_INTSTAT1        0x00//Interrupt Status1
#define MAX30102_INTSTAT2        0x01//Interrupt Status2
#define MAX30102_INTENABLE1      0x02//Interrupt Enable1
#define MAX30102_INTENABLE2      0x03//Interrupt Enable2
//FIFO Registers
#define MAX30102_FIFOWRITEPTR    0x04//FIFO Write Pointer, The FIFO Write Pointer points to the location where the MAX30102 writes the next sample. This pointer advances for each sample pushed on to the FIFO. It can also be changed through the I2C interface when MODE[2:0] is 010, 011, or 111.
#define MAX30102_FIFOOVERFLOW    0x05//FIFO Overflow Counter, When the FIFO is full, samples are not pushed on to the FIFO, samples are lost. OVF_COUNTER counts the number of samples lost. It saturates at 0x1F. When a complete sample is “popped” (i.e., removal of old FIFO data and shifting the samples down) from the FIFO (when the read pointer advances), OVF_COUNTER is reset to zero.
#define MAX30102_FIFOREADPTR     0x06//FIFO Read Pointer, The FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO through the I2C interface. This advances each time a sample is popped from the FIFO. The processor can also write to this pointer after reading the samples to allow rereading samples from the FIFO if there is a data communication error.
#define MAX30102_FIFODATA        0x07//FIFO Data Register, The circular FIFO depth is 32 and can hold up to 32 samples of data. The sample size depends on the number of LED channels (a.k.a. channels) configured as active. As each channel signal is stored as a 3-byte data signal, the FIFO width can be 3 bytes or 6 bytes in size.
//Configuration Registers
#define MAX30102_FIFOCONFIG      0x08//FIFO Configuration
#define MAX30102_MODECONFIG      0x09//Mode Configuration
#define MAX30102_PARTICLECONFIG  0x0A//SpO2 Configuration
#define MAX30102_LED1_PULSEAMP   0x0C//LED1 Pulse Amplitude
#define MAX30102_LED2_PULSEAMP   0x0D//LED2 Pulse Amplitude
#define MAX30102_LED3_PULSEAMP   0x0E//RESERVED
#define MAX30102_LED_PROX_AMP    0x10//RESERVED
#define MAX30102_MULTILEDCONFIG1 0x11//Multi-LED Mode Control Registers
#define MAX30102_MULTILEDCONFIG2 0x12//Multi-LED Mode Control Registers
//Die Temperature Registers
#define MAX30102_DIETEMPINT      0x1F//Die Temp Integer
#define MAX30102_DIETEMPFRAC     0x20//Die Temp Fraction
#define MAX30102_DIETEMPCONFIG   0x21//Die Temperature Config
#define MAX30102_PROXINTTHRESH   0x30//RESERVED
//Part ID Registers
#define MAX30102_REVISIONID      0xFE//Revision ID
#define MAX30102_PARTID          0xFF//Part ID:0x15
#define MAX30102_EXPECTED_PARTID  0x15

//存放传感器读数的循环缓冲区大小，不能小于2
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define MAX30102_SENSE_BUF_SIZE  2
#else
#define MAX30102_SENSE_BUF_SIZE  30
#endif


class DFRobot_MAX30102
{
//配置选项
//FIFO Configuration(寄存器地址0x08)
//sampleAverage(Table 3. Sample Averaging)
#define SAMPLEAVG_1     0
#define SAMPLEAVG_2     1
#define SAMPLEAVG_4     2
#define SAMPLEAVG_8     3
#define SAMPLEAVG_16    4
#define SAMPLEAVG_32    5

//Mode configuration(寄存器地址0x09)
//ledMode(Table 4. Mode Control)
#define MODE_REDONLY    2
#define MODE_RED_IR     3
#define MODE_MULTILED   7

//Particle sensing configuration(寄存器地址0x0A)
//adcRange(Table 5. SpO2 ADC Range Control)
#define ADCRANGE_2048   0
#define ADCRANGE_4096   1
#define ADCRANGE_8192   2
#define ADCRANGE_16384  3
//sampleRate(Table 6. SpO2 Sample Rate Control)
#define SAMPLERATE_50   0 
#define SAMPLERATE_100  1
#define SAMPLERATE_200  2
#define SAMPLERATE_400  3
#define SAMPLERATE_800  4
#define SAMPLERATE_1000 5
#define SAMPLERATE_1600 6
#define SAMPLERATE_3200 7
//pulseWidth(Table 7. LED Pulse Width Control)
#define PULSEWIDTH_69   0 
#define PULSEWIDTH_118  1
#define PULSEWIDTH_215  2
#define PULSEWIDTH_411  3

//Multi-LED Mode Control Registers(寄存器地址0x011)
#define SLOT_NONE       0
#define SLOT_RED_LED    1
#define SLOT_IR_LED     2

public:
  /*
    Interrupt Status(0x00–0x01) (pg 12)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |  A_FULL  | PPG_RDY  |  ALC_OVF |                    NONE                    | PWR_RDY  |
    * ------------------------------------------------------------------------------------------
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2   |     b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |                           NONE                                 |DIE_TEMP_RDY|   NONE   |
    * ------------------------------------------------------------------------------------------
  */
  /*
    Interrupt Enable(0x02–0x03) (pg 13)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |A_FULL_EN |PPG_RDY_EN|ALC_OVF_EN|                         NONE                          |
    * ------------------------------------------------------------------------------------------
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2   |       b1      |   b0  |
    * ------------------------------------------------------------------------------------------
    * |                           NONE                                 |DIE_TEMP_RDY_EN|  NONE |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   NONE:1;
    uint8_t   dieTemp:1;     // Internal Temperature Ready Flag
    uint8_t   NONE1:6;
    uint8_t   NONE2:5;
    uint8_t   ALCOverflow:1; // Ambient Light Cancellation Overflow
    uint8_t   dataReady:1;   // New FIFO Data Ready
    uint8_t   almostFull:1;  // FIFO Almost Full Flag
  } __attribute__ ((packed)) sEnable_t;
  /*
    FIFO Configuration(0x08) (pg 17)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4          | b3 |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |            SMP_AVE             |FIFO_ROLLOVER_EN|               FIFO_A_FULL            |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   almostFull:4; // FIFO Almost Full Value
    uint8_t   RollOver:1;   // FIFO Rolls on Full
    uint8_t   sampleAverag:3;  // Sample Averaging
  } __attribute__ ((packed)) sFIFO_t;

  /*
    Mode configuration(0x09) (pg 18)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |   SHDN   |   RESET  |             NONE               |              MODE               |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   ledMode:6; /*!< 010:Heart Rate mode, Red only. 011:SpO2 mode, Red and IR. 111:Multi-LED mode, Red and IR*/
    uint8_t   reset:1; /*!< 1:reset */
    uint8_t   shutDown:1; /*!< 0: wake up 1: put IC into low power mode*/
  } __attribute__ ((packed)) sMode_t;

  /*
    Particle sensing configuration(0x0A) (pg 18)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |   NONE   |     SPO2_ADC_RGE    |             SPO2_SR            |        LED_PW        |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   pulseWidth:2;
    uint8_t   sampleRate:3;
    uint8_t   adcRange:3;
  } __attribute__ ((packed)) sParticle_t;

  /*
    LED Pulse Amplitude(0x0C–0x0D) (pg 20)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |                                         LED1_PA                                        |
    * ------------------------------------------------------------------------------------------
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |                                         LED2_PA                                        |
    * ------------------------------------------------------------------------------------------
  */
  /*
    Multi-LED Mode Control Registers(0x011) (pg 21)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |   NONE   |              SLOT2             |   NONE   |             SLOT1               |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   SLOT1:4;
    uint8_t   SLOT2:4;
  } __attribute__ ((packed)) sMultiLED_t;

  /*!
   *@brief 保存样本的缓冲区
   */
  typedef struct {
    uint32_t red[MAX30102_SENSE_BUF_SIZE];
    uint32_t IR[MAX30102_SENSE_BUF_SIZE];
    uint8_t head;
    uint8_t tail;
  } sSenseBuf_t;

public:

  /*!
   *@brief 构造函数
   */
  DFRobot_MAX30102(void);

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
                           uint8_t ledMode = MODE_RED_IR, uint8_t sampleRate = SAMPLERATE_400, \
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

private:
  
  /*!
   *@brief 所有配置、阈值和数据寄存器复位。复位完成后，复位位自动清零
   */
  void softReset();

  /*!
   *@brief 进入省电模式。在省电模式下，所有寄存器都保留其值，读写操作可正常工作。所有中断都清除为零
   */
  void shutDown();

  /*!
   *@brief 唤醒模块，正常工作
   */
  void wakeUp();

  /*!
   *@brief 设置LED模式
   *@param mode 模式使用注释有ledMode的宏定义进行配置
   */
  void setLEDMode(uint8_t mode);
  
  /*!
   *@brief 设置ADC量程，默认4096 (nA)，15.63(pA) per LSB
   *@param adcRange ADC量程使用注释有adcRange的宏定义进行配置
   */
  void setADCRange(uint8_t adcRange);

  /*!
   *@brief 设置采样速率
   *@param sampleRate 采样速率使用注释有sampleRate的宏定义进行配置
   */
  void setSampleRate(uint8_t sampleRate);

  /*!
   *@brief 设置脉冲宽度，脉冲宽度越长，探测范围就越大
   *@param pulseWidth 脉冲宽度使用注释有pulseWidth的宏定义进行配置
   */
  void setPulseWidth(uint8_t pulseWidth);

  /*!
   *@brief 设置红灯的亮度
   *@param amplitude 振幅: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA
   */
  void setPulseAmplitudeRed(uint8_t amplitude);

  /*!
   *@brief 设置红外灯的亮度
   *@param amplitude 振幅: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA
   */
  void setPulseAmplitudeIR(uint8_t amplitude);

  /*!
   *@brief 根据给定编号配置led设备，一共有四个时隙，我们只用到slot1和slot2，设备有红光和红外光
   *@param slotNumber 槽编号，可取1,2
   *@param device LED设备名：SLOT_RED_LED 或 SLOT_IR_LED
   */
  void enableSlot(uint8_t slotNumber, uint8_t device);

  /*!
   *@brief 禁用所有槽
   */
  void disableAllSlots(void);

  /*!
   *@brief 启用 FIFO Almost Full Flag，当FIFO写指针有一定数量的空闲空间时，就会触发此中断
   */
  void enableAlmostFull(void);

  /*!
   *@brief 禁用 FIFO Almost Full Flag
   */
  void disableAlmostFull(void);

  /*!
   *@brief 启用 New FIFO Data Ready，当数据FIFO中有一个新样本时，就会触发此中断
   */
  void enableDataReady(void);

  /*!
   *@brief 禁用 New FIFO Data Ready
   */
  void disableDataReady(void);

  /*!
   *@brief 启用 Ambient Light Cancellation Overflow，当SpO2/HR光电二极管的环境光消除功能达到最大值时，就会触发此中断
   */
  void enableALCOverflow(void);

  /*!
   *@brief 禁用 Ambient Light Cancellation Overflow
   */
  void disableALCOverflow(void);

  /*!
   *@brief 启用 Internal Temperature Ready Flag，当内部模具温度转换完成时，会触发此中断
   */
  void enableDieTempReady(void);

  /*!
   *@brief 禁用 Internal Temperature Ready Flag
   */
  void disableDieTempReady(void);

  /*!
   *@brief 设置样本平均，传感器将会发送多个样本的平均值
   *@param samples 平均的样本数，使用注释有sampleAverage的宏定义进行配置
   */
  void setFIFOAverage(uint8_t samples);

  /*!
   *@brief 启用 FIFO Rolls on Full，如果FIFO满了，FIFO地址将归0,FIFO将继续填充新数据
   */
  void enableFIFORollover();

  /*!
   *@brief 禁用 FIFO Rolls on Full，如果FIFO满了，新样本将会丢失
   */
  void disableFIFORollover();

  /*!
   *@brief 指定空闲空间大小，使能中断后，达到设置的大小会触发'Almost Full'中断
   *@param numberOfSamples 空闲样本空间大小，空闲样本达到该值时，触发中断，如果设置为2则填30个样本会触发中断，设置为0则填32个样本会触发中断
   */
  void setFIFOAlmostFull(uint8_t numberOfSamples);

  /*!
   *@brief 读取芯片ID
   */
  uint8_t getPartID();

  /*!
   *@brief 得到FIFO写指针
   *@return 写指针
   */
  uint8_t getWritePointer(void);

  /*!
   *@brief 得到FIFO读指针
   *@return 读指针
   */
  uint8_t getReadPointer(void);

  /*!
   *@brief 重置FIFO
   */
  void resetFIFO(void);

  /*!
   *@brief 读取新数据并保存在结构体缓冲区
   */
  void getNewData(void);

  void writeReg(uint8_t reg, const void* pBuf, uint8_t size);
  uint8_t readReg(uint8_t reg, const void* pBuf, uint8_t size);

private:
  TwoWire *_pWire;
  uint8_t _i2cAddr;
  uint8_t _activeLEDs;
  sSenseBuf_t senseBuf;//存放多组数据的缓冲区
};

#endif