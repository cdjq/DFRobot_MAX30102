/*!
 * @file DFRobot_MAX30102.h
 * @brief Define the basic structure of class DFRobot_MAX30102
 * @n 这是一个有12路引脚的LED灯，实现了下面这些功能
 * @n 控制12路单色LED灯的亮度
 * @n 控制4路带RGB引脚的LED灯闪烁、亮度和变色，支持12V电源供电的LED灯，最高承受17V
 * @n 驱动可级联，每次收到新数据会将旧数据往后传递，每次发送N组数据锁存后可控制N个驱动
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V1.0
 * @date  2020-04-24
 * @https://github.com/DFRobot/DFRobot_MAX30102
 */
#ifndef _DFROBOT_MAX30102_H
#define _DFROBOT_MAX30102_H

#include <Arduino.h>
#include <Wire.h>

#define MAX30102_ADDRESS          0x57 //7-bit I2C Address
//Note that MAX30102 has the same I2C address and Part ID

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH
#elif defined(__SAMD21G18A__)
//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE
#else
#define I2C_BUFFER_LENGTH 32
#endif

//Status Registers
static const uint8_t MAX30102_INTSTAT1 =		0x00; //Interrupt Status1
static const uint8_t MAX30102_INTSTAT2 =		0x01; //Interrupt Status2
static const uint8_t MAX30102_INTENABLE1 =		0x02; //Interrupt Enable1
static const uint8_t MAX30102_INTENABLE2 =		0x03; //Interrupt Enable2

//FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPTR = 	0x04; //FIFO Write Pointer
static const uint8_t MAX30102_FIFOOVERFLOW = 	0x05; //FIFO Write Pointer
static const uint8_t MAX30102_FIFOREADPTR = 	0x06; //FIFO Read Pointer
static const uint8_t MAX30102_FIFODATA =		0x07; //FIFO Data Register

//Configuration Registers
static const uint8_t MAX30102_FIFOCONFIG = 		0x08; //FIFO Configuration
static const uint8_t MAX30102_MODECONFIG = 		0x09; //Mode Configuration
static const uint8_t MAX30102_PARTICLECONFIG = 	0x0A; //SpO2 Configuration
static const uint8_t MAX30102_LED1_PULSEAMP = 	0x0C; //LED1 Pulse Amplitude
static const uint8_t MAX30102_LED2_PULSEAMP = 	0x0D; //LED2 Pulse Amplitude
static const uint8_t MAX30102_LED3_PULSEAMP = 	0x0E; //RESERVED
//static const uint8_t MAX30102_LED_PROX_AMP = 	0x10; //
static const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11; //Multi-LED Mode Control Registers
static const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12; //Multi-LED Mode Control Registers

//Die Temperature Registers
static const uint8_t MAX30102_DIETEMPINT = 		0x1F; //Die Temp Integer
static const uint8_t MAX30102_DIETEMPFRAC = 	0x20; //Die Temp Fraction
static const uint8_t MAX30102_DIETEMPCONFIG = 	0x21; //Die Temperature Config

//Proximity Function Registers
//static const uint8_t MAX30102_PROXINTTHRESH = 	0x30; //

//Part ID Registers
static const uint8_t MAX30102_REVISIONID = 		0xFE; //Revision ID
static const uint8_t MAX30102_PARTID = 			0xFF; //Part ID:0x15

//Interrupt configuration (pg 13, 14)
//In SpO2 and HR modes, this interrupt triggers when the FIFO write pointer has a certain number of free spaces remaining
static const uint8_t MAX30102_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;
//In SpO2 and HR modes, this interrupt triggers when there is a new sample in the data FIFO
static const uint8_t MAX30102_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;
//This interrupt triggers when the ambient light cancellation function of the SpO2/HR photodiode has reached its maximum limit
static const uint8_t MAX30102_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

// static const uint8_t MAX30102_INT_PROX_INT_MASK = (byte)~0b00010000;
// static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
// static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

// static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
// static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
// static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

//Mode configuration commands (page 19)
static const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30102_SHUTDOWN = 		0x80;
static const uint8_t MAX30102_WAKEUP = 			0x00;

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_MODE_MASK = 		0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 	0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30102_MODE_MULTILED = 	0x07;

//Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;




class DFRobot_MAX30102
{
public:
  DFRobot_MAX30102(void);

  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30102_ADDRESS);

  uint32_t getRed(void); //Returns immediate red value
  uint32_t getIR(void); //Returns immediate IR value
  // uint32_t getGreen(void); //Returns immediate green value
  bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  //Configuration
  void softReset();
  void shutDown();
  void wakeUp();

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);
  void setPulseAmplitudeGreen(uint8_t value);
  // void setPulseAmplitudeProximity(uint8_t value);

  void setProximityThreshold(uint8_t threshMSB);

  //Multi-led configuration mode (page 22)
  void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
  void disableSlots(void);

  //Data Collection

  //Interrupts (page 13, 14)
  uint8_t getINT1(void); //Returns the main interrupt group
  uint8_t getINT2(void); //Returns the temp ready interrupt
  void enableAFULL(void); //Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  // void enablePROXINT(void);
  // void disablePROXINT(void);
  // void enableDIETEMPRDY(void);
  // void disableDIETEMPRDY(void);

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
  uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
  void setPROXINTTHRESH(uint8_t val);

  // //Die Temperature
  // float readTemperature();
  // float readTemperatureF();

  //Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();

  //Setup the IC with user selectable settings
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

  //Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

private:
  TwoWire *_i2cPort;
  uint8_t _i2caddr;
  byte activeLEDs; //在安装过程中设置。允许check()计算从FIFO中读取多少字节//激活的频道数是打开的频道数，可以是1到3个。2表示红色+IR

private:
  uint8_t revisionID;

  void readRevisionID();

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
};

#endif
