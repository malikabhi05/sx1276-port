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
#define XTAL_FREQ              32000000
#define RF_MID_BAND_THRESH     525000000

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

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

/*!
 * Radio FSK modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
}RadioFskSettings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
}RadioFskPacketHandler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
}RadioLoRaSettings_t;

/*!
 * Radio LoRa packet handler state
 */
typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;

/*!
 * Radio Settings
 */
typedef struct
{
    RadioState_t             State;
    RadioModems_t            Modem;
    uint32_t                 Channel;
    RadioFskSettings_t       Fsk;
    RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
}RadioSettings_t;

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
    RadioSettings_t          Settings;
}SX1276_t;

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

  //static uint8_t SX1276GetFskBandwidthRegValue( uint32_t bandwidth );

  void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous);

  void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout);

  uint32_t SX1276GetTimeOnAir(RadioModems_t modem, uint8_t pktLen);
