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

// registers - mostly lora specific
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_BITRATE_MSB          0x02
#define REG_BITRATE_LSB          0x03
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_LR_IRQ_FLAGS_MASK    0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_LR_SYMBTIMEOUT_LSB   0x1F
#define REG_LR_PKTSNRVALUE       0x19
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_LR_PAYLOADMAXLENGTH  0x23
#define REG_LR_HOPPERIOD         0x24
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_CONFIG          0x27
#define REG_SYNCVALUE1           0x28
#define REG_SYNCVALUE2           0x29
#define REG_SYNCVALUE3           0x2A
#define REG_RSSI_WIDEBAND        0x2c
#define REG_LR_TEST2F            0x2F
#define REG_PACKET_CONFIG1       0x30
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_FIFO_THRESH          0x35
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41
#define REG_VERSION              0x42
#define REG_LR_TEST30            0x30
#define REG_LR_DETECTOPTIMIZE    0x31
#define REG_LR_INVERTIQ          0x33
#define REG_LR_TEST36            0x36
#define REG_LR_DETECTIONTHRESHOLD                   0x37
#define REG_LR_SYNCWORD          0x39
#define REG_LR_TEST3A            0x3A
#define REG_LR_INVERTIQ2         0x3B
#define REG_IMAGE_CAL            0x3B
#define REG_LR_PLLHOP            0x44

#define RX_BUFFER_SIZE           256

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

/*!
 * RegDetectionThreshold
 */
#define RFLR_DETECTIONTHRESH_SF7_TO_SF12            0x0A // Default
#define RFLR_DETECTIONTHRESH_SF6                    0x0C

/*!
 * RegDetectOptimize
 */
#define RFLR_DETECTIONOPTIMIZE_MASK                 0xF8
#define RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12          0x03 // Default
#define RFLR_DETECTIONOPTIMIZE_SF6                  0x05

/*!
 * RegPllHop - lora
 */
#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default

/*!
 * RegModemConfig1 - lora
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default

/*!
 * RegModemConfig2 - lora
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F
#define RFLR_MODEMCONFIG2_SF_6                      0x60
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80
#define RFLR_MODEMCONFIG2_SF_9                      0x90
#define RFLR_MODEMCONFIG2_SF_10                     0xA0
#define RFLR_MODEMCONFIG2_SF_11                     0xB0
#define RFLR_MODEMCONFIG2_SF_12                     0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK         0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON           0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF          0x00 // Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default

/*!
 * RegModemConfig3 - lora
 */
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK  0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON    0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK              0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON                0x04 // Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF 0x00

// reg image cal related WOW!!
/*!
 * RegImageCal
 */
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default

#define RF_IMAGECAL_TEMPCHANGE_HIGHER               0x08
#define RF_IMAGECAL_TEMPCHANGE_LOWER                0x00

#define RF_IMAGECAL_TEMPTHRESHOLD_MASK              0xF9
#define RF_IMAGECAL_TEMPTHRESHOLD_05                0x00
#define RF_IMAGECAL_TEMPTHRESHOLD_10                0x02  // Default
#define RF_IMAGECAL_TEMPTHRESHOLD_15                0x04
#define RF_IMAGECAL_TEMPTHRESHOLD_20                0x06

#define RF_IMAGECAL_TEMPMONITOR_MASK                0xFE
#define RF_IMAGECAL_TEMPMONITOR_ON                  0x00 // Default
#define RF_IMAGECAL_TEMPMONITOR_OFF                 0x01

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80
#define RFLR_IRQFLAGS_RXDONE                        0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10
#define RFLR_IRQFLAGS_TXDONE                        0x08
#define RFLR_IRQFLAGS_CADDONE                       0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02
#define RFLR_IRQFLAGS_CADDETECTED                   0x01

/*!
 * RegInvertIQ
 */
#define RFLR_INVERTIQ_RX_MASK                       0xBF
#define RFLR_INVERTIQ_RX_OFF                        0x00
#define RFLR_INVERTIQ_RX_ON                         0x40
#define RFLR_INVERTIQ_TX_MASK                       0xFE
#define RFLR_INVERTIQ_TX_OFF                        0x01
#define RFLR_INVERTIQ_TX_ON                         0x00

/*!
 * RegInvertIQ2
 */
#define RFLR_INVERTIQ2_ON                           0x19
#define RFLR_INVERTIQ2_OFF                          0x1D

/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default

// register masks
/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01


// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_LONG_RANGE_MODE_OFF 0x7F
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07
#define OP_MODE_MASK             0x80

// PA config
#define PA_BOOST                 0x80

// IRQ masks
//#define IRQ_TX_DONE_MASK           0x08
//#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
//#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH             255

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

  void SX1276Send(uint8_t *buffer, uint8_t size);

  void SX1276SetTx(uint32_t timeout);

  void SX1276SetRx(uint32_t timeout);

  void SX1276StartCad(void);

  void SX1276SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time);

  void SX1276SetMaxPayloadLength(RadioModems_t modem, uint8_t max);

  void SX1276SetPublicNetwork(bool enable);

  void SX1276OnTimeoutIrq(void);

  void SX1276OnDio0Irq(void);

  void SX1276OnDio1Irq(void);

  void SX1276OnDio2Irq(void);
