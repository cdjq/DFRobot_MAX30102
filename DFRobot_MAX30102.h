#ifndef _DFROBOT_MAX30102_H
#define _DFROBOT_MAX30102_H

#include <Arduino.h>
#include <Wire.h>

//Open this macro to see the detailed running process of the program 
#define ENABLE_DBG
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#define MAX30102_IIC_ADDRESS          0x57 //7-bit I2C Address

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//定义I2C缓冲区的大小
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define I2C_BUFFER_LENGTH BUFFER_LENGTH //Wire.h
#elif defined(__SAMD21G18A__)
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE //RingBuffer.h
#else
#define I2C_BUFFER_LENGTH 32
#endif

//Status Registers
#define MAX30102_INTSTAT1 0x00//Interrupt Status1
#define MAX30102_INTSTAT2 0x01//Interrupt Status2
#define MAX30102_INTENABLE1 0x02//Interrupt Enable1
#define MAX30102_INTENABLE2 0x03//Interrupt Enable2
//FIFO Registers
#define MAX30102_FIFOWRITEPTR   0x04//FIFO Write Pointer
#define MAX30102_FIFOOVERFLOW   0x05//FIFO Write Pointer
#define MAX30102_FIFOREADPTR   0x06//FIFO Read Pointer
#define MAX30102_FIFODATA   0x07//FIFO Data Register
//Configuration Registers
#define MAX30102_FIFOCONFIG    0x08//FIFO Configuration
#define MAX30102_MODECONFIG    0x09//Mode Configuration
#define MAX30102_PARTICLECONFIG   0x0A//SpO2 Configuration
#define MAX30102_LED1_PULSEAMP   0x0C//LED1 Pulse Amplitude
#define MAX30102_LED2_PULSEAMP   0x0D//LED2 Pulse Amplitude
#define MAX30102_LED3_PULSEAMP   0x0E//RESERVED
#define MAX30102_LED_PROX_AMP   0x10//RESERVED
#define MAX30102_MULTILEDCONFIG1  0x11//Multi-LED Mode Control Registers
#define MAX30102_MULTILEDCONFIG2  0x12//Multi-LED Mode Control Registers
//Die Temperature Registers
#define MAX30102_DIETEMPINT    0x1F//Die Temp Integer
#define MAX30102_DIETEMPFRAC   0x20//Die Temp Fraction
#define MAX30102_DIETEMPCONFIG   0x21//Die Temperature Config
#define MAX30102_PROXINTTHRESH   0x30//RESERVED
//Part ID Registers
#define MAX30102_REVISIONID    0xFE//Revision ID
#define MAX30102_PARTID     0xFF//Part ID:0x15
#define MAX30102_EXPECTED_PARTID  0x15

//FIFO Configuration(0x08)
#define MAX30102_SAMPLEAVG_1  0
#define MAX30102_SAMPLEAVG_2  1
#define MAX30102_SAMPLEAVG_4  2
#define MAX30102_SAMPLEAVG_8  3
#define MAX30102_SAMPLEAVG_16 4
#define MAX30102_SAMPLEAVG_32 5
//Mode configuration(0x09)
#define MAX30102_MODE_REDONLY   2
#define MAX30102_MODE_RED_IR    3
#define MAX30102_MODE_MULTILED  7
//Particle sensing configuration(0x0A)
#define MAX30102_ADCRANGE_2048   0
#define MAX30102_ADCRANGE_4096   1
#define MAX30102_ADCRANGE_8192   2
#define MAX30102_ADCRANGE_16384  3
#define MAX30102_SAMPLERATE_50   0
#define MAX30102_SAMPLERATE_100  1
#define MAX30102_SAMPLERATE_200  2
#define MAX30102_SAMPLERATE_400  3
#define MAX30102_SAMPLERATE_800  4
#define MAX30102_SAMPLERATE_1000 5
#define MAX30102_SAMPLERATE_1600 6
#define MAX30102_SAMPLERATE_3200 7
#define MAX30102_PULSEWIDTH_69   0
#define MAX30102_PULSEWIDTH_118  1
#define MAX30102_PULSEWIDTH_215  2
#define MAX30102_PULSEWIDTH_411  3
//Multi-LED Mode Control Registers(0x011)
#define MAX30102_SLOT_NONE  		0
#define MAX30102_SLOT_RED_LED  	1
#define MAX30102_SLOT_IR_LED  	2

class DFRobot_MAX30102
{
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

public:
  /*!
   *@brief 构造函数
   */
  DFRobot_MAX30102(void);
  /*!
   *@brief 初始化
   *@param amplitude 振幅: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA 
   *@return 初始化成功返回true，失败返回false
   */
  bool begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2cAddr = MAX30102_IIC_ADDRESS);
  /*!
   *@brief 获得red值
   *@return 4字节的无符号red值
   */
  uint32_t getRed(void);
  /*!
   *@brief 获得IR值
   *@return 4字节的无符号IR值
   */
  uint32_t getIR(void);
  /*!
   *@brief 在给定最长时间范围里，检查新数据
   *@param waitTime 发现数据的最长时间
   *@return 发现新数据返回true，没有数据返回false
   */
  bool foundData(uint8_t waitTime);

  void reset();
  void shutDown();
  void wakeUp();

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
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
   *@brief 给定槽号，为其配置led设备
   *@param slotNumber 槽编号，可取1,2
   *@param device LED设备名：MAX30102_SLOT_RED_LED 或 MAX30102_SLOT_IR_LED
   */
  void enableSlot(uint8_t slotNumber, uint8_t device); 
  /*!
   *@brief 关闭所有槽
   */
  void disableAllSlots(void);
  /*!
   *@brief 启用 FIFO Almost Full Flag，当FIFO写指针有一定数量的空闲空间时，就会触发这个中断
   */
  void enableAlmostFull(void);
  /*!
   *@brief 禁用 FIFO Almost Full Flag
   */
  void disableAlmostFull(void);
  /*!
   *@brief 启用 New FIFO Data Ready，当数据FIFO中有一个新样本时，就会触发这个中断
   */
  void enableDataReady(void);
  /*!
   *@brief 禁用 New FIFO Data Ready
   */
  void disableDataReady(void);
  /*!
   *@brief 启用 Ambient Light Cancellation Overflow，当SpO2/HR光电二极管的环境光对消功能达到最大极限时，就会触发这个中断
   */
  void enableALCOverflow(void);
  /*!
   *@brief 禁用 Ambient Light Cancellation Overflow
   */
  void disableALCOverflow(void);
  void enableDieTempReady(void);
  void disableDieTempReady(void);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);

  //FIFO Reading
  uint16_t check(void); //Checks for new data and fills FIFO
  uint8_t available(void); //Tells caller how many new samples are available (head - tail)
  void nextSample(void); //Advances the tail of the sense array
  uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void); //Sets the read/write pointers to zero

  //Die Temperature
  float readTemperature();
  float readTemperatureF();

  //Detecting ID/Revision
  uint8_t readPartID();

  //Setup the IC with user selectable settings
  void setup(uint8_t ledBrightness = 0x1F, uint8_t sampleAverage = MAX30102_SAMPLEAVG_4, uint8_t ledMode = MAX30102_MODE_RED_IR, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

  //Low-level I2C communication
  void writeReg(uint8_t reg, const void* pBuf, uint8_t size);
  uint8_t readReg(uint8_t reg, const void* pBuf, uint8_t size);
  void writeReg8(uint8_t reg, uint8_t value);
  uint8_t readReg8(uint8_t reg);

private:
  TwoWire *_pWire;
  uint8_t _i2cAddr;
  uint8_t activeLEDs; //活动的LED数
  sEnable_t enableReg;
  sFIFO_t FIFOReg;
  sMode_t modeReg;
  sParticle_t particleReg;
  sMultiLED_t multiLEDReg;
};

#endif