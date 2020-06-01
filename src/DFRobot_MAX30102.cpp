/*!
 * @file DFRobot_MAX30102.h
 * @brief Define the basic structure of class DFRobot_MAX30102
 * @n 这是一个血氧饱和度和心率监测模块
 * @n 可以采集红光和红外光读数，温度传感器读数
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

bool DFRobot_MAX30102::begin(TwoWire *pWire, uint8_t i2cAddr)
{
  _i2cAddr = i2cAddr;
  _pWire = pWire;
  _pWire->begin();
  // 检查模块连接
  if (!getPartID() == MAX30102_EXPECTED_PARTID) {
    // 读取的part ID与预期的part ID不匹配。
    DBG("not expected partid");
    return false;
  }
  //复位
  softReset();
  senseBuf.tail = 0;
  senseBuf.head = 0;
  return true;
}

///状态相关配置

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
  //循环等待传感器回应，直到复位变回0，才算完成重置
  uint32_t startTime = millis();
  while (millis() - startTime < 100) {
    readReg(MAX30102_MODECONFIG, &modeReg, 1);
    if (modeReg.reset == 0) break; //完成
    delay(1);
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

void DFRobot_MAX30102::resetFIFO(void)
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

///FIFO配置

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

void DFRobot_MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples)//触发该中断的样本数 = 32 - numberOfSamples
{
  sFIFO_t FIFOReg;
  readReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.almostFull = numberOfSamples;
  writeReg(MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

///获取温度

float DFRobot_MAX30102::readTemperatureC()//返回温度用摄氏度表示
{
  //使能
  uint8_t byteTemp = 0x01;
  writeReg(MAX30102_DIETEMPCONFIG, &byteTemp, 1);
  //循环等待传感器回应，确保已经使能后才开始读
  uint32_t startTime = millis();
  while (millis() - startTime < 100) { //超时时间
    readReg(MAX30102_DIETEMPCONFIG, &byteTemp, 1);
    if ((byteTemp & 0x01) == 0) break; //完成
    delay(1);
  }

  //读出模具温度整数部分
  uint8_t tempInt;
  readReg(MAX30102_DIETEMPINT, &tempInt, 1);
  //读出模具温度小数部分
  uint8_t tempFrac;
  readReg(MAX30102_DIETEMPFRAC, &tempFrac, 1);

  return (float)tempInt + ((float)tempFrac * 0.0625);
}

float DFRobot_MAX30102::readTemperatureF()
{
  float temp = readTemperatureC();
  if (temp != -999.0) temp = temp * 1.8 + 32.0;
  return (temp);
}

uint8_t DFRobot_MAX30102::getPartID()
{
  uint8_t byteTemp;
  readReg(MAX30102_PARTID, &byteTemp, 1);
  return byteTemp;
}


void DFRobot_MAX30102::sensorConfiguration(uint8_t ledBrightness, uint8_t sampleAverage, uint8_t ledMode, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange)
{
  /*FIFO设置*/
  //设置芯片将多个样本平均
  setFIFOAverage(sampleAverage);

  /*传感器相关设置*/
  setADCRange(adcRange);
  //每秒取样本数
  setSampleRate(sampleRate);
  //脉冲宽度越长，探测范围就越大，在69us 0.4mA时，大约2英寸，在411us 0.4mA时，大约6英寸
  setPulseWidth(pulseWidth);
  //LED亮度
  setPulseAmplitudeRed(ledBrightness);
  setPulseAmplitudeIR(ledBrightness);
  //每个样本被分割成四个时间槽，SLOT1~SLOT4，根据设置的LED模式确定
  enableSlot(1, MAX30102_SLOT_RED_LED);//将Slot1的RED设置为活动
  if (ledMode > MAX30102_MODE_REDONLY) enableSlot(2, MAX30102_SLOT_IR_LED);//将Slot2的IR设置为活动

  /*模式设置*/
  setLEDMode(ledMode);
  //根据设置的LED模式确定激活的LED灯数量
  if (ledMode == MAX30102_MODE_REDONLY) {
    _activeLEDs = 1;
  } else {
    _activeLEDs = 2;
  }

  enableFIFORollover(); //启用FIFO满时，自动归零
  resetFIFO(); //重置FIFO，准备之后的读数
}

///获取数据

uint32_t DFRobot_MAX30102::getRed(void)
{
  getNewData();
  return (senseBuf.red[senseBuf.head]);
}

uint32_t DFRobot_MAX30102::getIR(void)
{
  getNewData();
  return (senseBuf.IR[senseBuf.head]);
}

void DFRobot_MAX30102::getNewData(void)//循环获取新数据
{
  int32_t numberOfSamples = 0;
  uint8_t readPointer = 0;//读取FIFO读指针
  uint8_t writePointer = 0;
  while (1) {//缓冲区有可用样本后，才会返回，避免缓冲区数据未更新而出现相同的读数
    readPointer = getReadPointer();//读取FIFO读指针
    writePointer = getWritePointer();
    //判断有无可读数据
    if (readPointer == writePointer) {
      DBG("no data");
    } else {
      numberOfSamples = writePointer - readPointer;
      if (numberOfSamples < 0) numberOfSamples += 32;//待读取样本数量
      int32_t bytesNeedToRead = numberOfSamples * _activeLEDs * 3;//待读取字节数
        //从传感器获取相应字节数
        while (bytesNeedToRead > 0) {
          senseBuf.head++;
          senseBuf.head %= MAX30102_SENSE_BUF_SIZE;//指向新数据的指针
          uint32_t tempBuf = 0;
          if (_activeLEDs > 1) { //如果启用了IR，读6个字节
            uint8_t temp[6];
            uint8_t tempex;

            readReg(MAX30102_FIFODATA, temp, 6);
            //存储空间上顺序反了，交换顺序
            for(uint8_t i = 0; i < 3; i++){
              tempex = temp[i];
              temp[i] = temp[5-i];
              temp[5-i] = tempex;
            }

            memcpy(&tempBuf, temp, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.IR[senseBuf.head] = tempBuf;
            memcpy(&tempBuf, temp+3, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.red[senseBuf.head] = tempBuf;
          } else { 
            uint8_t temp[3];
            uint8_t tempex;

            //读3个字节，对应RED
            readReg(MAX30102_FIFODATA, temp, 3);
            tempex = temp[0];
            temp[0] = temp[2];
            temp[2] = tempex;

            memcpy(&tempBuf, temp, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.red[senseBuf.head] = tempBuf;
          }
          bytesNeedToRead -= _activeLEDs * 3;
        }
      
      return;
    }
    delay(1);
  }
}

uint8_t DFRobot_MAX30102::numberOfSamples(void)//计算缓冲区中可用样本数
{
  int8_t numberOfSamples = senseBuf.head - senseBuf.tail;
  if (numberOfSamples < 0) numberOfSamples += MAX30102_SENSE_BUF_SIZE;
  return numberOfSamples;//返回可用样本数
}

void DFRobot_MAX30102::heartrateAndOxygenSaturation(int32_t* SPO2,int8_t* SPO2Valid,int32_t* heartRate,int8_t* heartRateValid)
{
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Arduino Uno
  uint16_t irBuffer[100];
  uint16_t redBuffer[100];
#else
  uint32_t irBuffer[100];
  uint32_t redBuffer[100];
#endif
  int32_t bufferLength = 100;
  //装满缓冲区
  for (uint8_t i = 0 ; i < bufferLength ; i++) {
    getNewData(); //读取数据，存放在缓冲区
    redBuffer[i] = senseBuf.red[senseBuf.head];
    irBuffer[i] = senseBuf.IR[senseBuf.head];
  }

  //计算bufferLength个样本的心率和血氧饱和度
  maxim_heart_rate_and_oxygen_saturation(/**pun_ir_buffer=*/irBuffer, /*n_ir_buffer_length=*/bufferLength, /**pun_red_buffer=*/redBuffer, \
      /**pn_spo2=*/SPO2, /**pch_spo2_valid=*/SPO2Valid, /**pn_heart_rate=*/heartRate, /**pch_hr_valid=*/heartRateValid);
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