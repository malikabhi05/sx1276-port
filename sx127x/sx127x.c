#include "sx127x.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

SX1276_t SX1276;

void LoRaClass()
  //_spiSettings(10E6, MSBFIRST, SPI_MODE0),
  //_ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  //_frequency(0),
  //_packetIndex(0),
  //_implicitHeaderMode(0),
  //_onReceive(NULL)
{
    // initialize MRAA
    int mraa_rv;
    if((mraa_rv = mraa_init()) != MRAA_SUCCESS) {
        printf("%s: mraa_init failed (%d).\n", __FUNCTION__, mraa_rv);
    }

    if(!(SX1276.spi = mraa_spi_init(0)))
        printf("Unable to Initialize Spi bus\n");

    if(mraa_spi_frequency(SX1276.spi, 4000000) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set higher frequency\n");
    }

    SX1276._frequency  = 0;
    SX1276._packetIndex = 0;
    SX1276._implicitHeaderMode = 0;

    SX1276._ss = LORA_DEFAULT_SS_PIN;
    SX1276._reset = LORA_DEFAULT_RESET_PIN;
    SX1276._dio0 = LORA_DEFAULT_DIO0_PIN;
}

int begin(long frequency)
{
  // setup pins
  //pinMode(_ss, OUTPUT);
  //pinMode(_reset, OUTPUT);

    // Initialize the cs pin
    if(!(SX1276.gpio_ss = mraa_gpio_init(SX1276._ss))) {
        printf("sx1276: unable to init SPI CS\n");
    }

    if(mraa_gpio_dir(SX1276.gpio_ss, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        printf("sx1276: unable to set direction on SPI CS\n");
    }

    // initialize the reset pin
    if(!(SX1276.gpio_reset = mraa_gpio_init(SX1276._reset))) {
        printf("sx1276: unable to init chip reset\n");
    }

    if(mraa_gpio_dir(SX1276.gpio_reset, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        printf("sx1276: unable to set direction on reset pin\n");
    }


  // perform reset
  //digitalWrite(_reset, LOW);
  //delay(10);
  //digitalWrite(_reset, HIGH);
  //delay(10);

    mraa_gpio_write(SX1276.gpio_reset, 1);
    upm_delay_us(1000);
    mraa_gpio_write(SX1276.gpio_reset, 0);
    upm_delay_us(1000);
    mraa_gpio_write(SX1276.gpio_reset, 1);
    upm_delay_us(15000);

    dumpRegisters();

  // set SS high
  //digitalWrite(_ss, HIGH);
    mraa_gpio_write(SX1276.gpio_ss, 1);

  // start SPI
  // it has already begun

    // the following code can be used as is hopefully

    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        printf("wrong version\n");
        //return 0;
    }
    printf("chip version: %x\n", version);

    // put in sleep mode
    SX1276sleep();

    // set frequency
    setFrequency(frequency);

    printf("setting rx and tx base addresses\n");
    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    printf("REG_FIFO_TX_BASE_ADDR: %x\n", readRegister(REG_FIFO_TX_BASE_ADDR));
    printf("REG_FIFO_RX_BASE_ADDR: %x\n", readRegister(REG_FIFO_RX_BASE_ADDR));

    // set LNA boost
    printf("before REG_LNA: %x\n", readRegister(REG_LNA));
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x23);
    printf("after REG_LNA: %x\n", readRegister(REG_LNA));

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
    printf("REG_MODEM_CONFIG_3; %x\n", readRegister(REG_MODEM_CONFIG_3));

    // set output power to 17 dBm
    // second argument is the default pin here
    setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

    // put in standby mode
    idle();

    return 1;
}

void end() {
    // should work as is hopefully
    // put in sleep mode
    SX1276sleep();

    // stop SPI
    //SPI.end();
}

int beginPacket(int implicitHeader) {
    // as per driver this is by default false
  
    // as is
    // put in standby mode
    idle();
    printf("inside begin packet\n");

    if (implicitHeader) {
        implicitHeaderMode();
    } else {
        explicitHeaderMode();
    }

    printf("initial value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_FIFO_ADDR_PTR));
    printf("initial value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_PAYLOAD_LENGTH));
    // reset FIFO address and paload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    printf("later value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_FIFO_ADDR_PTR));
    printf("later value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_PAYLOAD_LENGTH));

    return 1;
}

int endPacket() {
    // again as is hopefully

    printf("value of REG_IRQ_FLAGS very prior to clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));

    printf("inside end packet\n");
    printf("final value of REG_PAYLOAD_LENGTH: %x\n", readRegister(REG_PAYLOAD_LENGTH));

    // put in TX mode
    printf("initial value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
    //printf("value being written: %x\n", );
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    printf("next value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));

    // wait for TX done
    while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

    printf("interrupt 0 detected\n");

    printf("value of REG_IRQ_FLAGS prior to clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));
    // clear IRQ's
    //writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    writeRegister(REG_IRQ_FLAGS, 0x80);
    printf("value of REG_IRQ_FLAGS after clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));

    return 1;
}

// size is set to 0 by default
int parsePacket(int size) {
    // as is hopefully again

    int packetLength = 0;
    int irqFlags = readRegister(REG_IRQ_FLAGS);
    //if(irqFlags != 0)
    //    printf("reception: %x\n", irqFlags);

    if (size > 0) {
        implicitHeaderMode();
        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        SX1276._packetIndex = 0;

        // read packet length
        if (SX1276._implicitHeaderMode) {
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        } else {
            packetLength = readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        idle();
    } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int packetRssi()
{
    return (readRegister(REG_PKT_RSSI_VALUE) - (SX1276._frequency < 868E6 ? 164 : 157));
}

float packetSnr()
{
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

// commenting these functions out for now, if important then will implement them
/*
int write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}
*/
int write_buf(const uint8_t *buffer, size_t size)
{
    printf("aa\n");
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);

    printf("current length: %d\n", currentLength);
    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    printf("bb\n");
    // write data
    for (size_t i = 0; i < size; i++) {
        writeRegister(REG_FIFO, buffer[i]);
    }

    printf("cc\n");
    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    printf("dd\n");

    return size;
}

int available()
{
    return (readRegister(REG_RX_NB_BYTES) - SX1276._packetIndex);
}

int SX1276read() {
    if (!available()) {
        return -1;
    }

    SX1276._packetIndex++;

    return readRegister(REG_FIFO);
}

int peek() {
    if (!available()) {
        return -1;
    }

    // store current FIFO address
    int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = readRegister(REG_FIFO);

    // restore FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void flush() {
}

// not using is currently, will implement if needed
/*
void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    writeRegister(REG_DIO_MAPPING_1, 0x00);

    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}
*/

// the default size here is 0
void receive(int size) {
    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }

    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void idle() {
    printf("previous value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    printf("next value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
}

void SX1276sleep() {
    printf("Sleep function\n");
    printf("before: REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    printf("after: REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
}

// output pin default is: PA_OUTPUT_PA_BOOST_PIN
void setTxPower(int level, int outputPin) {
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level < 0) {
            level = 0;
        } else if (level > 14) {
            level = 14;
        }

        printf("value to be written to REG_PA_CONFIG; %x\n", 0x70 | level);
        writeRegister(REG_PA_CONFIG, 0x70 | level);
        printf("REG_PA_CONFIG: %x\n", readRegister(REG_PA_CONFIG));
    } else {
        // PA BOOST
        if (level < 2) {
            level = 2;
        } else if (level > 17) {
            level = 17;
        }

        printf("value to be written to REG_PA_CONFIG; %x\n", PA_BOOST | (level - 2));
        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
        printf("REG_PA_CONFIG: %x\n", readRegister(REG_PA_CONFIG));
    }
}

void setFrequency(long frequency) {

    //frequency = frequency/1.00001732945;
    SX1276._frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

    printf("REG_FRF_MSB: %x\n", readRegister(REG_FRF_MSB));
    printf("REG_FRF_MID: %x\n", readRegister(REG_FRF_MID));
    printf("REG_FRF_LSB: %x\n", readRegister(REG_FRF_LSB));
}

void setSpreadingFactor(int sf) {
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    if (sf == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void setSignalBandwidth(long sbw) {
    int bw;

    if (sbw <= 7.8E3) {
        bw = 0;
    } else if (sbw <= 10.4E3) {
        bw = 1;
    } else if (sbw <= 15.6E3) {
        bw = 2;
    } else if (sbw <= 20.8E3) {
        bw = 3;
    } else if (sbw <= 31.25E3) {
        bw = 4;
    } else if (sbw <= 41.7E3) {
        bw = 5;
    } else if (sbw <= 62.5E3) {
        bw = 6;
    } else if (sbw <= 125E3) {
        bw = 7;
    } else if (sbw <= 250E3) {
        bw = 8;
    } else /*if (sbw <= 250E3)*/ {
        bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void setCodingRate4(int denominator) {
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }

    int cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void setPreambleLength(long length) {
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setSyncWord(int sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void crc() {
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void noCrc() {
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

int8_t random() {
    return readRegister(REG_RSSI_WIDEBAND);
}

void setPins(int ss, int reset, int dio0) {
    SX1276._ss = ss;
    SX1276._reset = reset;
    SX1276._dio0 = dio0;
}

// already done inside the init function (class/begin function for this code)
/*
void setSPIFrequency(uint32_t frequency){
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}
*/

void dumpRegisters()
{
  for (int i = 0; i < 128; i++) {
    //printf("0x");
    //printf("%x",i);
    //printf(": 0x");
    printf("0x%x: 0x%x\n", i, readRegister(i));
    //printf(readRegister(i), HEX);
  }
}

void explicitHeaderMode() {
    //printf("inside explicit header mode\n");
    SX1276._implicitHeaderMode = 0;

    //printf("initial value of REG_MODEM_CONFIG_1: %x\n", readRegister(REG_MODEM_CONFIG_1));
    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
    //printf("next value of REG_MODEM_CONFIG_1: %x\n", readRegister(REG_MODEM_CONFIG_1));
}

void implicitHeaderMode() {
    printf("inside implicit header mode\n");
    SX1276._implicitHeaderMode = 1;

    printf("initial value of REG_MODEM_CONFIG_1: %x\n", readRegister(REG_MODEM_CONFIG_1));
    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
    printf("next value of REG_MODEM_CONFIG_1: %x\n", readRegister(REG_MODEM_CONFIG_1));
}

void handleDio0Rise() {
    int irqFlags = readRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        SX1276._packetIndex = 0;

        // read packet length
        int packetLength = SX1276._implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

// _onreceive not currently implemented
/*
        if (_onReceive) {
            _onReceive(packetLength);
        }
*/
        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);
    }
}

uint8_t readRegister(uint8_t address) {
    //return singleTransfer(address & 0x7f, 0x00);

    uint8_t tx_buf[2] = { address & 0x7f, 0 };
    uint8_t rx_buf[2];

    mraa_gpio_write(SX1276.gpio_ss, 0);
    if (mraa_spi_transfer_buf(SX1276.spi, tx_buf, rx_buf, 2) != MRAA_SUCCESS) {
        printf("Unable to transfer data over the SPI bus\n");
        printf("SPI operation failed\n");
        mraa_gpio_write(SX1276.gpio_ss, 1);
        return -1;
    }
    mraa_gpio_write(SX1276.gpio_ss, 1);

    return rx_buf[1];
}

void writeRegister(uint8_t address, uint8_t value) {
    //singleTransfer(address | 0x80, value);

    uint8_t pkt[2] = {(uint8_t)(address | 0x80), value};

    mraa_gpio_write(SX1276.gpio_ss, 0);
    if (mraa_spi_transfer_buf(SX1276.spi, pkt, NULL, 2) != MRAA_SUCCESS) {
        printf("Unable to transfer data over the SPI bus\n");
        printf("SPI operation failed\n");
        mraa_gpio_write(SX1276.gpio_ss, 1);
    }
    mraa_gpio_write(SX1276.gpio_ss, 1);
}


// not sure what to do here as of now, maybe fifo requires its own function
// will have to see
/*
uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  digitalWrite(_ss, LOW);

  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}
*/

// not currently implemented
/*
void LoRaClass::onDio0Rise()
{
  LoRa.handleDio0Rise();
}
*/
