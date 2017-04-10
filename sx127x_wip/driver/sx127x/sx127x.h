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

#define FREQ_STEP              61.03515625

// init values, use, no idea
#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_FIFO_ADDR_PTR      , 0x1E },\
    { MODEM_FSK , REG_FIFO_TX_BASE_ADDR  , 0xD2 },\
    { MODEM_FSK , REG_PKT_RSSI_VALUE     , 0x01 },\
    { MODEM_FSK , REG_PREAMBLE_DETECT    , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNC_CONFIG        , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKET_CONFIG1     , 0xD8 },\
    { MODEM_FSK , REG_FIFO_THRESH        , 0x8F },\
    { MODEM_FSK , REG_IMAGE_CAL          , 0x02 },\
    { MODEM_FSK , REG_DIO_MAPPING_1      , 0x00 },\
    { MODEM_FSK , REG_DIO_MAPPING_2      , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
} 

typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA
} RadioModems_t;



//class LoRaClass : public Stream {
//public:
  void LoRaClass();

  int init(long frequency);
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

  void SX1276SetModem(RadioModems_t modem);

  void SX1276SetOpMode(uint8_t mode);

  static void onDio0Rise();

  void SX1276RxChainCalibration();

  bool SX1276IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh);
  uint32_t SX1276Random( void );

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

