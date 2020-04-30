/*!
 * @file DFRobot_MAX30102.h
 * @brief Define the basic structure of class DFRobot_MAX30102
 * @n 这是一个血氧饱和度和心率监测模块
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V1.0
 * @date  2020-03-30
 * @https://github.com/DFRobot/DFRobot_MAX30102
 */
#include <DFRobot_MAX30102.h>
//最多存储32个样本，这是微控制器的附加本地存储
const int STORAGE_SIZE = 4;
struct Record {
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
} sense; //这是传感器读数的循环缓冲

DFRobot_MAX30102::DFRobot_MAX30102(void)
{

}

bool DFRobot_MAX30102::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2cAddr)
{
  _pWire = &wirePort; //Grab which port the user wants us to use
  _pWire->begin();
  _pWire->setClock(i2cSpeed);

  _i2cAddr = i2cAddr;

  // 检查模块连接
  if (!readPartID() == MAX30102_EXPECTED_PARTID) {
    // 读取的part ID与预期的part ID不匹配。
    return false;
  }
  return true;
}

// Configuration

void DFRobot_MAX30102::enableAlmostFull(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.almostFull = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableAlmostFull(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.almostFull = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::enableDataReady(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dataReady = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableDataReady(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dataReady = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::enableALCOverflow(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.ALCOverflow = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableALCOverflow(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.ALCOverflow = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}


void DFRobot_MAX30102::enableDieTempReady(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dieTemp = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableDieTempReady(void)
{
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dieTemp = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}


void DFRobot_MAX30102::reset(void)
{
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.reset = 1;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
  //循环清除数据，然后完成重置，超时100ms自动退出
  uint32_t startTime = millis();
  while (millis() - startTime < 100) {
    readReg(MAX30102_MODECONFIG, &modeReg, 1);
    if (modeReg.reset == 0) break; //完成
    delay(1); //不要让I2C总线负担过重
  }
}

void DFRobot_MAX30102::shutDown(void)
{
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.shutDown = 1;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::wakeUp(void)
{
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.shutDown = 0;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::setLEDMode(uint8_t ledMode)
{
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.ledMode = ledMode;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::setADCRange(uint8_t adcRange)
{
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.adcRange = adcRange;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setSampleRate(uint8_t sampleRate)
{
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.sampleRate = sampleRate;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setPulseWidth(uint8_t pulseWidth)
{
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.pulseWidth = pulseWidth;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setPulseAmplitudeRed(uint8_t amplitude)
{
  writeReg8(MAX30102_LED1_PULSEAMP, amplitude);
}

void DFRobot_MAX30102::setPulseAmplitudeIR(uint8_t amplitude)
{
  writeReg8(MAX30102_LED2_PULSEAMP, amplitude);
}


//选择槽和启用LED设备
void DFRobot_MAX30102::enableSlot(uint8_t slotNumber, uint8_t device)
{
  switch (slotNumber) {
  case (1):
    readReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    multiLEDReg.SLOT1 = device;
    writeReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    break;
  case (2):
    readReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    multiLEDReg.SLOT2 = device;
    writeReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    break;
  default:
    break;
  }
}

void DFRobot_MAX30102::disableAllSlots(void)
{
  multiLEDReg.SLOT1 = 0;
  multiLEDReg.SLOT2 = 0;
  writeReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
}

// FIFO Configuration

//Set sample average (Table 3, Page 18)
void DFRobot_MAX30102::setFIFOAverage(uint8_t numberOfSamples)
{
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.sampleAverag = numberOfSamples;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void DFRobot_MAX30102::clearFIFO(void)
{
  writeReg8(MAX30102_FIFOWRITEPTR, 0);
  writeReg8(MAX30102_FIFOOVERFLOW, 0);
  writeReg8(MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void DFRobot_MAX30102::enableFIFORollover(void)
{
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.RollOver = 1;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

//Disable roll over if FIFO over flows
void DFRobot_MAX30102::disableFIFORollover(void)
{
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.RollOver = 0;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void DFRobot_MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples)
{
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.almostFull = numberOfSamples;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

//Read the FIFO Write Pointer
uint8_t DFRobot_MAX30102::getWritePointer(void)
{
  return (readReg8(MAX30102_FIFOWRITEPTR));
}



// Die Temperature
// Returns temp in C
float DFRobot_MAX30102::readTemperature()
{
  // Step 1: Config die temperature register to take 1 temperature sample
  writeReg8(MAX30102_DIETEMPCONFIG, 0x01);
  writeReg8(MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  uint32_t startTime = millis();
  while (millis() - startTime < 100) {
    uint8_t response = readReg8(MAX30102_DIETEMPCONFIG);
    if ((response & 0x01) == 0) break; //完成
    delay(1); 
  }

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readReg8(MAX30102_DIETEMPINT);
  uint8_t tempFrac = readReg8(MAX30102_DIETEMPFRAC);

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float DFRobot_MAX30102::readTemperatureF()
{
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// // Set the PROX_INT_THRESHold
// void DFRobot_MAX30102::setPROXINTTHRESH(uint8_t val)
// {
//   writeReg8(MAX30102_PROXINTTHRESH, val);
// }


uint8_t DFRobot_MAX30102::readPartID()
{
  return readReg8(MAX30102_PARTID);
}





//Setup the sensor
//The DFRobot_MAX30102 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the DFRobot_MAX30102 sensor
void DFRobot_MAX30102::setup(uint8_t ledBrightness, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
  reset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == MAX30102_SAMPLEAVG_1) setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == MAX30102_SAMPLEAVG_2) setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == MAX30102_SAMPLEAVG_4) setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == MAX30102_SAMPLEAVG_8) setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == MAX30102_SAMPLEAVG_16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == MAX30102_SAMPLEAVG_32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over

  //Mode Configuration
  if (ledMode == MAX30102_MODE_MULTILED) {
    setLEDMode(MAX30102_MODE_MULTILED); //Watch all LED channels
    activeLEDs = 2; 
  } else if (ledMode == MAX30102_MODE_RED_IR) {
    setLEDMode(MAX30102_MODE_RED_IR); //Red and IR
    activeLEDs = 2; 
  } else {
    setLEDMode(MAX30102_MODE_REDONLY); //Red only
    activeLEDs = 1; 
  }

  //Particle Sensing Configuration
  if(adcRange < 4096) setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30102_SAMPLERATE_50); //每秒取50个样本
  else if (sampleRate < 200) setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30102_SAMPLERATE_3200);
  else setSampleRate(MAX30102_SAMPLERATE_50);

  //脉冲宽度越长，探测范围就越大，在69us 0.4mA时，大约2英寸，在411us 0.4mA时，大约6英寸
  if (pulseWidth < 118) setPulseWidth(MAX30102_PULSEWIDTH_69);
  else if (pulseWidth < 215) setPulseWidth(MAX30102_PULSEWIDTH_118); 
  else if (pulseWidth < 411) setPulseWidth(MAX30102_PULSEWIDTH_215); 
  else if (pulseWidth == 411) setPulseWidth(MAX30102_PULSEWIDTH_411);
  else setPulseWidth(MAX30102_PULSEWIDTH_69);

  //LED亮度配置
  setPulseAmplitudeRed(ledBrightness);
  setPulseAmplitudeIR(ledBrightness);
  //每个样本被分割成四个时间槽，SLOT1到SLOT4
  enableSlot(1, MAX30102_SLOT_RED_LED);//将Slot1的RED设置为活动
  if (ledMode > MAX30102_MODE_REDONLY) enableSlot(2, MAX30102_SLOT_IR_LED);//将Slot2的IR设置为活动

  clearFIFO(); //重置FIFO，准备之后的读数
}

// Data Collection
//Tell caller how many samples are available
uint8_t DFRobot_MAX30102::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value
uint32_t DFRobot_MAX30102::getRed(void)
{
  if(foundData(250)) //花费250ms找数据
    return (sense.red[sense.head]);
  else
    return 0; //没有发现数据
}

//Report the most recent IR value
uint32_t DFRobot_MAX30102::getIR(void)
{
  if(foundData(250)) //花费250ms找数据
    return (sense.IR[sense.head]);
  else
    return 0; //没有发现数据
}


// //报告FIFO中的下一个红色值
// uint32_t DFRobot_MAX30102::getFIFORed(void)
// {
//   return (sense.red[sense.tail]);
// }

// //Report the next IR value in the FIFO
// uint32_t DFRobot_MAX30102::getFIFOIR(void)
// {
//   return (sense.IR[sense.tail]);
// }

// //Advance the tail
// void DFRobot_MAX30102::nextSample(void)
// {
//   if(available()) { //Only advance the tail if new data is available
//     sense.tail++;
//     sense.tail %= STORAGE_SIZE; //Wrap condition
//   }
// }

//轮询传感器以获取新数据
//如果有新的数据可用，将更新结构体中的head和tail
//返回发现的的新样本数
uint16_t DFRobot_MAX30102::check(void)
{
  
  uint8_t readPointer = readReg8(MAX30102_FIFOREADPTR);//Read the FIFO Read Pointer
  uint8_t writePointer = getWritePointer();

  int numberOfSamples = 0;
  //读取寄存器数据直到FIFO_RD_PTR = FIFO_WR_PTR
  if (readPointer != writePointer) {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    _pWire->beginTransmission(MAX30102_IIC_ADDRESS);
    _pWire->write(MAX30102_FIFODATA);
    _pWire->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0) {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH) {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      _pWire->requestFrom(MAX30102_IIC_ADDRESS, toGet);

      while (toGet > 0) {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into int32_t
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = _pWire->read();
        temp[1] = _pWire->read();
        temp[0] = _pWire->read();

        //Convert array to int32_t
        memcpy(&tempLong, temp, sizeof(tempLong));

        tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1) {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = _pWire->read();
          temp[1] = _pWire->read();
          temp[0] = _pWire->read();

          //Convert array to int32_t
          memcpy(&tempLong, temp, sizeof(tempLong));

          tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.IR[sense.head] = tempLong;
        }
        toGet -= activeLEDs * 3;
      }
    }
  } 
  return (numberOfSamples); //Let the world know how much new data we found
}

bool DFRobot_MAX30102::foundData(uint8_t waitTime)
{
  uint8_t startTime = millis();
  while(millis() - startTime > waitTime) {
    if(check() == true) 
      return true; //找到新数据
    delay(1);
  }
  return false;//超时
}



void DFRobot_MAX30102::writeReg(uint8_t reg, const void* pBuf, uint8_t size)
{
    if(pBuf == NULL){
        DBG("pBuf ERROR!! : null pointer");
    }
    uint8_t * _pBuf = (uint8_t *)pBuf;
    _pWire->beginTransmission(_i2cAddr);
    _pWire->write(&reg, 1);
    
    for(uint16_t i = 0; i < size; i++){
        _pWire->write(_pBuf[i]);
    }
    _pWire->endTransmission();
}

uint8_t DFRobot_MAX30102::readReg(uint8_t reg, const void* pBuf, uint8_t size)
{
    if(pBuf == NULL){
        DBG("pBuf ERROR!! : null pointer");
    }
    uint8_t * _pBuf = (uint8_t *)pBuf;
    _pWire->beginTransmission(_i2cAddr);
    _pWire->write(&reg, 1);
    
    if( _pWire->endTransmission() != 0){
        return 0;
    }

    _pWire->requestFrom(_i2cAddr,  size);
    for(uint16_t i = 0; i < size; i++){
        _pBuf[i] = _pWire->read();
    }
    return size;
}

void DFRobot_MAX30102::writeReg8(uint8_t reg, uint8_t value) {
  _pWire->beginTransmission(_i2cAddr);
  _pWire->write(reg);
  _pWire->write(value);
  _pWire->endTransmission();
}

uint8_t DFRobot_MAX30102::readReg8(uint8_t reg)
{
  _pWire->beginTransmission(_i2cAddr);
  _pWire->write(reg);
  _pWire->endTransmission(false);
  _pWire->requestFrom((uint8_t)_i2cAddr, (uint8_t)1); // 请求一个字节
  if (_pWire->available()) {
    return _pWire->read();//返回读取的一个字节的内容
  }
  return 0; //失败
}