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
  DFRobot_MAX30102 Breakout: Output all the raw Red/IR/Green readings, check INT pin and interrupt register
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30102_Breakout

  Outputs all Red/IR/Green values as fast as possible
  Checks the interrupt pin to see if an interrupt occurred
  Checks the interrupt register to see if a bit was set

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

DFRobot_MAX30102 particleSensor;

long startTime;
long samplesTaken = 0; //Counter for calculating the Hz or read rate

// byte interruptPin = 3; //Connect INT pin on breakout board to pin 3

void setup()
{
  // pinMode(interruptPin, INPUT);

  Serial.begin(115200);
  Serial.println("Initializing...");

  //Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("Failed to initialize the particle Sensor");
    while (1);
  }

  //让我们将传感器配置为快速运行，这样就可以超过缓冲区并导致中断
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.enableAFULL(); //Enable the almost full interrupt (default is 32 samples)
  
  particleSensor.setFIFOAlmostFull(3); //Set almost full int to fire at 29 samples

  startTime = millis();
}

void loop()
{
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) //do we have new data?
  {
    samplesTaken++;

    Serial.print(" R[");
    Serial.print(particleSensor.getRed());
    Serial.print("] IR[");
    Serial.print(particleSensor.getIR());
    Serial.print("] Hz[");
    Serial.print((float)samplesTaken / ((millis() - startTime) / 1000.0), 2);
    Serial.print("]");

    if (digitalRead(interruptPin) == LOW) //Hardware way of reading interrupts
    {
      Serial.print(" INT!");
    }

    byte flags = particleSensor.getINT1(); //Software way of reading interrupts
    if (flags)
    {
      Serial.print(" I[");
      Serial.print(flags, BIN);
      Serial.print("]");
    }

    Serial.println();

    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
}
