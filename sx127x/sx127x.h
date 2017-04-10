#pragma once

//#include <Arduino.h>
//#include <SPI.h>

#include "upm.h"
#include "mraa/gpio.h"
#include "mraa/spi.h"
#include <math.h>
#include <string.h>

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

//class LoRaClass : public Stream {
//public:
  void LoRaClass();

  int begin(long frequency);
  void end();

  int beginPacket();
  int endPacket();

  int parsePacket(int size);
  int packetRssi();
  float packetSnr();

  // from Print
  //int write(uint8_t byte);
  int write_buf(const uint8_t *buffer, size_t size);

  // from Stream
  int available();
  int SX1276read();
  int peek();
  void flush();

  void onReceive(void(*callback)(int));

  void receive(int size);
  void idle();
  void SX1276sleep();

  void setTxPower(int level, int outputPin);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void crc();
  void noCrc();

  int8_t random();

  void setPins(int ss, int reset, int dio0);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters();

//private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

/*
private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
*/

typedef struct SX1276_s
{
    mraa_gpio_context        gpio_ss;
    mraa_gpio_context        gpio_reset;
    mraa_gpio_context        gpio_dio0;
    uint8_t                  _ss;
    uint8_t                  _reset;
    uint8_t                  _dio0;
    mraa_gpio_context        DIO1;
    mraa_spi_context         spi;
    int                      _frequency;
    int                      _packetIndex;
    int                      _implicitHeaderMode;
}SX1276_t;
//};

//extern LoRaClass LoRa;

