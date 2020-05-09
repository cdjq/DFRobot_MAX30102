/*!
 * @file DFRobot_MAX30102.h
 * @brief Define the basic structure of class DFRobot_MAX30102
 * @n 这是一个血氧饱和度和心率监测模块
 * @n 可以采集红色传感器和红外传感器读数，温度传感器读数
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [YeHangYu](hangyu.ye@dfrobot.com)
 * @version  V1.0
 * @date  2020-03-30
 * @https://github.com/DFRobot/DFRobot_MAX30102
 */
#include <DFRobot_MAX30102.h>


DFRobot_MAX30102::DFRobot_MAX30102(void)
{

}

bool DFRobot_MAX30102::begin(TwoWire *pWire, uint32_t i2cSpeed, uint8_t i2cAddr)
{
  _i2cAddr = i2cAddr;
  _pWire = pWire;
  _pWire->begin();
  _pWire->setClock(i2cSpeed);
  // 检查模块连接
  if (!readPartID() == MAX30102_EXPECTED_PARTID) {
    // 读取的part ID与预期的part ID不匹配。
    return false;
  }
  return true;
}

void DFRobot_MAX30102::enableAlmostFull(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.almostFull = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableAlmostFull(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.almostFull = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::enableDataReady(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dataReady = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableDataReady(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dataReady = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::enableALCOverflow(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.ALCOverflow = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableALCOverflow(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.ALCOverflow = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::enableDieTempReady(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dieTemp = 1;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}
void DFRobot_MAX30102::disableDieTempReady(void)
{
  sEnable_t enableReg;
  readReg(MAX30102_INTENABLE1, &enableReg, 2);
  enableReg.dieTemp = 0;
  writeReg(MAX30102_INTENABLE1, &enableReg, 2);
}

void DFRobot_MAX30102::softReset(void)
{
  sMode_t modeReg;
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.reset = 1;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
  //循环等待传感器回应，直到复位变回0，才算完成重置，超时100ms自动退出
  uint32_t startTime = millis();
  while (millis() - startTime < 100) {
    readReg(MAX30102_MODECONFIG, &modeReg, 1);
    if (modeReg.reset == 0) break; //完成
    delay(1); //不要让I2C总线负担过重
  }
}

void DFRobot_MAX30102::shutDown(void)
{
  sMode_t modeReg;
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.shutDown = 1;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::wakeUp(void)
{
  sMode_t modeReg;
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.shutDown = 0;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::setLEDMode(uint8_t ledMode)
{
  sMode_t modeReg;
  readReg(MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.ledMode = ledMode;
  writeReg(MAX30102_MODECONFIG, &modeReg, 1);
}

void DFRobot_MAX30102::setADCRange(uint8_t adcRange)
{
  sParticle_t particleReg;
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.adcRange = adcRange;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setSampleRate(uint8_t sampleRate)
{
  sParticle_t particleReg;
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.sampleRate = sampleRate;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setPulseWidth(uint8_t pulseWidth)
{
  sParticle_t particleReg;
  readReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.pulseWidth = pulseWidth;
  writeReg(MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void DFRobot_MAX30102::setPulseAmplitudeRed(uint8_t amplitude)
{
  uint8_t byteTemp = amplitude;
  writeReg(MAX30102_LED1_PULSEAMP, &byteTemp, 1);
}

void DFRobot_MAX30102::setPulseAmplitudeIR(uint8_t amplitude)
{
  uint8_t byteTemp = amplitude;
  writeReg(MAX30102_LED2_PULSEAMP, &byteTemp, 1);
}

void DFRobot_MAX30102::enableSlot(uint8_t slotNumber, uint8_t device)
{
  sMultiLED_t multiLEDReg;
  switch (slotNumber) {//选择槽和启用LED设备
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
  sMultiLED_t multiLEDReg;
  multiLEDReg.SLOT1 = 0;
  multiLEDReg.SLOT2 = 0;
  writeReg(MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
}

void DFRobot_MAX30102::clearFIFO(void)
{
  uint8_t byteTemp = 0;
  writeReg(MAX30102_FIFOWRITEPTR, &byteTemp, 1);
  writeReg(MAX30102_FIFOOVERFLOW, &byteTemp, 1);
  writeReg(MAX30102_FIFOREADPTR, &byteTemp, 1);
}

uint8_t DFRobot_MAX30102::getWritePointer(void)//得到FIFO写指针
{
  uint8_t byteTemp;
  readReg(MAX30102_FIFOWRITEPTR, &byteTemp, 1);
  return byteTemp;
}
uint8_t DFRobot_MAX30102::getReadPointer(void)//得到FIFO读指针
{
  uint8_t byteTemp;
  readReg(MAX30102_FIFOREADPTR, &byteTemp, 1);
  return byteTemp;
}

/// FIFO Configuration

void DFRobot_MAX30102::setFIFOAverage(uint8_t numberOfSamples)
{
  sFIFO_t FIFOReg;
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.sampleAverag = numberOfSamples;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void DFRobot_MAX30102::enableFIFORollover(void)
{
  sFIFO_t FIFOReg;
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.RollOver = 1;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void DFRobot_MAX30102::disableFIFORollover(void)
{
  sFIFO_t FIFOReg;
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.RollOver = 0;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void DFRobot_MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples)//样本数设置: 0x00 is 32 samples, 0x02 is 30 samples
{
  sFIFO_t FIFOReg;
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.almostFull = numberOfSamples;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

/// Die Temperature

float DFRobot_MAX30102::readTemperature()//返回温度用摄氏度表示
{
  // Config die temperature register to take 1 temperature sample
  uint8_t byteTemp = 0x01;
  writeReg(MAX30102_DIETEMPCONFIG, &byteTemp, 1);
  //循环等待传感器回应
  uint32_t startTime = millis();
  while (millis() - startTime < 100) { //超时时间100ms
    uint8_t response;
    readReg(MAX30102_DIETEMPCONFIG, &response, 1);
    if ((response & 0x01) == 0) break; //完成
    delay(1);
  }

  // Read die temperature register
  uint8_t tempInt;
  readReg(MAX30102_DIETEMPINT, &tempInt, 1);
  uint8_t tempFrac;
  readReg(MAX30102_DIETEMPFRAC, &tempFrac, 1);

  // Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

float DFRobot_MAX30102::readTemperatureF()
{
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

uint8_t DFRobot_MAX30102::readPartID()
{
  uint8_t byteTemp;
  readReg(MAX30102_PARTID, &byteTemp, 1);
  return byteTemp;
}


void DFRobot_MAX30102::sensorConfiguration(uint8_t ledBrightness, uint8_t sampleAverage, uint8_t ledMode, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange)
{
  softReset();
  //FIFO Configuration
  //设置芯片将多个样本平均
  if (sampleAverage == MAX30102_SAMPLEAVG_1) setFIFOAverage(MAX30102_SAMPLEAVG_1); //不求平均值
  else if (sampleAverage == MAX30102_SAMPLEAVG_2) setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == MAX30102_SAMPLEAVG_4) setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == MAX30102_SAMPLEAVG_8) setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == MAX30102_SAMPLEAVG_16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == MAX30102_SAMPLEAVG_32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //设置为30个样本来触发一个'Almost Full'中断
  enableFIFORollover(); //启用FIFO满时，自动归零

  //Mode Configuration
  if (ledMode == MAX30102_MODE_MULTILED) {
    setLEDMode(MAX30102_MODE_MULTILED);
    activeLEDs = 2;
  } else if (ledMode == MAX30102_MODE_RED_IR) {
    setLEDMode(MAX30102_MODE_RED_IR);
    activeLEDs = 2;
  } else {
    setLEDMode(MAX30102_MODE_REDONLY);
    activeLEDs = 1;
  }

  //Particle Sensing Configuration
  if(adcRange < MAX30102_ADCRANGE_4096) setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < MAX30102_ADCRANGE_8192) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < MAX30102_ADCRANGE_16384) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == MAX30102_ADCRANGE_16384) setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < MAX30102_SAMPLERATE_100) setSampleRate(MAX30102_SAMPLERATE_50); //每秒取50个样本
  else if (sampleRate < MAX30102_SAMPLERATE_200) setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < MAX30102_SAMPLERATE_400) setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < MAX30102_SAMPLERATE_800) setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < MAX30102_SAMPLERATE_1000) setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < MAX30102_SAMPLERATE_1600) setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < MAX30102_SAMPLERATE_3200) setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == MAX30102_SAMPLERATE_3200) setSampleRate(MAX30102_SAMPLERATE_3200);
  else setSampleRate(MAX30102_SAMPLERATE_50);

  //脉冲宽度越长，探测范围就越大，在69us 0.4mA时，大约2英寸，在411us 0.4mA时，大约6英寸
  if (pulseWidth < MAX30102_PULSEWIDTH_118) setPulseWidth(MAX30102_PULSEWIDTH_69);
  else if (pulseWidth < MAX30102_PULSEWIDTH_215) setPulseWidth(MAX30102_PULSEWIDTH_118);
  else if (pulseWidth < MAX30102_PULSEWIDTH_411) setPulseWidth(MAX30102_PULSEWIDTH_215);
  else if (pulseWidth == MAX30102_PULSEWIDTH_411) setPulseWidth(MAX30102_PULSEWIDTH_411);
  else setPulseWidth(MAX30102_PULSEWIDTH_69);

  //LED亮度配置
  setPulseAmplitudeRed(ledBrightness);
  setPulseAmplitudeIR(ledBrightness);
  //每个样本被分割成四个时间槽，SLOT1~SLOT4，根据设置的LED模式启用槽
  enableSlot(1, MAX30102_SLOT_RED_LED);//将Slot1的RED设置为活动
  if (ledMode > MAX30102_MODE_REDONLY) enableSlot(2, MAX30102_SLOT_IR_LED);//将Slot2的IR设置为活动

  clearFIFO(); //重置FIFO，准备之后的读数
}

///获取数据

uint32_t DFRobot_MAX30102::getRed(void)
{
  if(foundData(250)) //花费250ms找数据
    return (senseBuf.red[senseBuf.head]);
  else
    return 0; //没有发现数据
}

uint32_t DFRobot_MAX30102::getIR(void)
{
  if(foundData(250)) //花费250ms找数据
    return (senseBuf.IR[senseBuf.head]);
  else
    return 0; //没有发现数据
}

//循环获取新数据
//如果有新的数据可用，将更新结构体中的head和tail
//返回发现的的新样本数
bool DFRobot_MAX30102::foundData(uint8_t waitTime)
{
  //循环等待传感器回应
  uint8_t startTime = millis();
  while(millis() - startTime > waitTime) {
    uint8_t readPointer = getReadPointer();//读取FIFO读指针
    uint8_t writePointer = getWritePointer();
    int32_t numberOfSamples = 0;
    //读取寄存器数据直到FIFO_RD_PTR = FIFO_WR_PTR
    if (readPointer != writePointer) {
      //计算我们需要从传感器获得的读数数量
      numberOfSamples = writePointer - readPointer;
      if (numberOfSamples < 0) numberOfSamples += 32; //样本数

      //有了读取的数量，现在需要读取的字节，对于本例，我们只使用Red和IR(各3个字节)
      int32_t bytesLeftToRead = numberOfSamples * activeLEDs * 3;

      //准备从FIFO寄存器读取一组数据
      _pWire->beginTransmission(MAX30102_IIC_ADDRESS);
      _pWire->write(MAX30102_FIFODATA);
      _pWire->endTransmission();

      //可能需要读取多达288字节的数据，块的大小不能大于I2C_BUFFER_LENGTH，需要读多次
      while (bytesLeftToRead > 0) {
        int32_t toGet = bytesLeftToRead;
        if (toGet > I2C_BUFFER_LENGTH) {
          //如果一次读取6个字节（red和IR各3字节），因此必须进行取余，抛弃剩下的几个字节
          toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3));
        }
        bytesLeftToRead -= toGet;
        //从传感器获取相应字节数
        _pWire->requestFrom(MAX30102_IIC_ADDRESS, toGet);
        while (toGet > 0) {
          senseBuf.head++;
          senseBuf.head %= MAX30102_SENSE_BUF_SIZE;//指向新数据的指针
          uint8_t temp[sizeof(uint32_t)];//用一个字节的数组表示4字节的整型数
          uint32_t tempLong;

          //读3个字节，对应RED
          temp[3] = 0;
          temp[2] = _pWire->read();
          temp[1] = _pWire->read();
          temp[0] = _pWire->read();
          //转换为uint32_t
          memcpy(&tempLong, temp, sizeof(tempLong));
          tempLong &= 0x3FFFF; //3字节有效
          senseBuf.red[senseBuf.head] = tempLong;

          if (activeLEDs > 1) { //如果启用了IR
            //再读3个字节，对应IR
            temp[3] = 0;
            temp[2] = _pWire->read();
            temp[1] = _pWire->read();
            temp[0] = _pWire->read();
            //转换为uint32_t
            memcpy(&tempLong, temp, sizeof(tempLong));
            tempLong &= 0x3FFFF; //3字节有效
            senseBuf.IR[senseBuf.head] = tempLong;
          }
          toGet -= activeLEDs * 3;
        }
      }
    }
    if(numberOfSamples != 0) {
      return true;//找到新数据
    }
    delay(1);
  }
  return false;//超时
}

uint8_t DFRobot_MAX30102::available(void)//计算缓冲区中可用样本数
{
  int8_t numberOfSamples = senseBuf.head - senseBuf.tail;
  if (numberOfSamples < 0) numberOfSamples += MAX30102_SENSE_BUF_SIZE;
  return numberOfSamples;//返回可用样本数
}

void DFRobot_MAX30102::nextSample(void)//指向缓冲区中的下一个样本
{
  if(available()) { //还有新数据
    senseBuf.tail++;
    senseBuf.tail %= MAX30102_SENSE_BUF_SIZE;
  }
}

uint32_t DFRobot_MAX30102::getFIFORed(void)
{
  return (senseBuf.red[senseBuf.tail]);
}

uint32_t DFRobot_MAX30102::getFIFOIR(void)
{
  return (senseBuf.IR[senseBuf.tail]);
}

void DFRobot_MAX30102::writeReg(uint8_t reg, const void* pBuf, uint8_t size)
{
  if(pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_i2cAddr);
  _pWire->write(&reg, 1);

  for(uint16_t i = 0; i < size; i++) {
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

uint8_t DFRobot_MAX30102::readReg(uint8_t reg, const void* pBuf, uint8_t size)
{
  if(pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_i2cAddr);
  _pWire->write(&reg, 1);

  if( _pWire->endTransmission() != 0) {
    return 0;
  }

  _pWire->requestFrom(_i2cAddr,  size);
  for(uint16_t i = 0; i < size; i++) {
    _pBuf[i] = _pWire->read();
  }
  return size;
}
