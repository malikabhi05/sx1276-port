#include "sx127x.h"
#include "sx127x_fsk.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

uint8_t rx_done = 0;

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx127x.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

static SX1276_t SX1276;

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

int init(long frequency)
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
    mraa_gpio_write(SX1276.gpio_reset, 1);
    upm_delay_us(1000);
    mraa_gpio_write(SX1276.gpio_reset, 0);
    upm_delay_us(1000);
    mraa_gpio_write(SX1276.gpio_reset, 1);
    upm_delay_us(15000);

    //dumpRegisters();
    SX1276RxChainCalibration();

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

    // initialize a bunch of registers here
    // no clue why this is needed, the driver should work without this too
    // but putting it in because its in the semtech/upm driver
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        writeRegister( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem(MODEM_LORA);

    // set frequency - to a default
    setFrequency(frequency);

    printf("setting rx and tx base addresses\n");
    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    printf("REG_FIFO_TX_BASE_ADDR: %x\n", readRegister(REG_FIFO_TX_BASE_ADDR));
    printf("REG_FIFO_RX_BASE_ADDR: %x\n", readRegister(REG_FIFO_RX_BASE_ADDR));

    // set LNA boost
    //printf("before REG_LNA: %x\n", readRegister(REG_LNA));
    //writeRegister(REG_LNA, readRegister(REG_LNA) | 0x23);
    //printf("after REG_LNA: %x\n", readRegister(REG_LNA));

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
    //printf("inside begin packet\n");

/*
    if (implicitHeader) {
        implicitHeaderMode();
    } else {
        explicitHeaderMode();
    }
*/

    //printf("initial value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_FIFO_ADDR_PTR));
    //printf("initial value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_PAYLOAD_LENGTH));
    // reset FIFO address and paload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    //printf("later value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_FIFO_ADDR_PTR));
    //printf("later value of REG_FIFO_ADDR_PTR: %x\n", readRegister(REG_PAYLOAD_LENGTH));

    return 1;
}

int endPacket() {
    // again as is hopefully

    printf("value of REG_IRQ_FLAGS very prior to clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));

    //printf("inside end packet\n");
    //printf("final value of REG_PAYLOAD_LENGTH: %x\n", readRegister(REG_PAYLOAD_LENGTH));

    // put in TX mode
    // replace this with the set opmode call
    //writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    SX1276SetOpMode(MODE_TX);

    // wait for TX done
    while((readRegister(REG_IRQ_FLAGS) & RFLR_IRQFLAGS_TXDONE) == 0);

    printf("interrupt 0 detected\n");

    //printf("value of REG_IRQ_FLAGS prior to clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));
    // clear IRQ's
    //writeRegister(REG_IRQ_FLAGS, RFLR_IRQFLAGS_TXDONE);
    writeRegister(REG_IRQ_FLAGS, 0x80);
    //printf("value of REG_IRQ_FLAGS after clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));

    return 1;
}

// size is set to 0 by default
int parsePacket(int size) {
    // as is hopefully again

    int packetLength = 0;
    int irqFlags = readRegister(REG_IRQ_FLAGS);
    //if(irqFlags == 0)
    //    printf("reception: %x\n", irqFlags);

/*
    if (size > 0) {
        implicitHeaderMode();
        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }
*/

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);
    //upm_delay_us(100000);

    if ((irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) && (irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == 0) {
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
    //} else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)) {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        // again using set opmode function
        // MODE_RX_CONTINUOUS
        //SX1276SetOpMode(MODE_RX_SINGLE);
        SX1276SetOpMode(MODE_RX_CONTINUOUS);
        //writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int packetRssi(RadioModems_t modem)
{
    int rssi = 0;
    switch(modem) {
        case MODEM_FSK:
            rssi = -(readRegister(REG_PKT_RSSI_VALUE) >> 1);
            break;
        case MODEM_LORA:
            rssi = readRegister(REG_PKT_RSSI_VALUE) - (SX1276._frequency < 868E6 ? 164 : 157);
            break;
        default:
            rssi = -1;
            break;
    }
    //return (readRegister(REG_PKT_RSSI_VALUE) - (SX1276._frequency < 868E6 ? 164 : 157));
    return rssi;
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
    //printf("aa\n");
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);

    printf("current length: %d\n", currentLength);
    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    //printf("bb\n");
    // write data
    for (size_t i = 0; i < size; i++) {
        //printf("writing buffer : %x\n", buffer[i]);
        writeRegister(REG_FIFO, buffer[i]);
    }

    //printf("cc\n");
    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    //printf("written to fifo and payload len set: %d\n", readRegister(REG_PAYLOAD_LENGTH));

    //printf("dd\n");

    return size;
}

int available()
{
    return (readRegister(REG_RX_NB_BYTES) - SX1276._packetIndex);
}

int SX1276read() {
/*
    if (!available()) {
        return -1;
    }
*/

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
/*
    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }
*/

    //writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    SX1276SetOpMode(MODE_RX_CONTINUOUS);
}

void idle() {
    // setting to idle
    //printf("Setting to idle state\n");
    //printf("previous value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
    //writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    SX1276SetOpMode(MODE_STDBY);
    //printf("next value of REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
}

void SX1276sleep() {
    printf("Setting to sleep/ FIFO not accessible\n");
    //printf("before: REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
    //writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    SX1276SetOpMode(MODE_SLEEP);
    //printf("after: REG_OP_MODE: %x\n", readRegister(REG_OP_MODE));
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

    printf("setting Frequency to: %d\n", frequency);
    SX1276._frequency = frequency;
    SX1276.Settings.Channel = frequency;
    frequency = frequency/1.00001732945;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

    //printf("REG_FRF_MSB: %x\n", readRegister(REG_FRF_MSB));
    //printf("REG_FRF_MID: %x\n", readRegister(REG_FRF_MID));
    //printf("REG_FRF_LSB: %x\n", readRegister(REG_FRF_LSB));
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

    if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == 0) {
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

// taken from the semtech driver
// modified
void SX1276SetModem(RadioModems_t modem) {
    SX1276.Settings.Modem = modem;
    switch(modem) {
        // need to be careful here, this driver explicitly sets the mode to LORA
        default:
        case MODEM_LORA:
            SX1276sleep();
            writeRegister(REG_OP_MODE, readRegister(REG_OP_MODE) | MODE_LONG_RANGE_MODE);

            // this part is right out of the driver
            writeRegister(REG_DIO_MAPPING_1, 0x00);
            writeRegister(REG_DIO_MAPPING_2, 0x00);
            break;
        case MODEM_FSK:
            SX1276sleep();
            writeRegister(REG_OP_MODE, readRegister(REG_OP_MODE) & MODE_LONG_RANGE_MODE_OFF);

            // this part is right out of the driver
            writeRegister(REG_DIO_MAPPING_1, 0x00);
            writeRegister(REG_DIO_MAPPING_2, 0x30);    // DIO5 = ModeReady
            break;
    }
}

void SX1276SetOpMode(uint8_t mode) {
    writeRegister(REG_OP_MODE, (readRegister(REG_OP_MODE)&OP_MODE_MASK) | mode);
}

void SX1276RxChainCalibration() {
    printf("Entering rx chain calibration\n");
    uint8_t regPaConfigInitVal;
    uint8_t initialFreq_msb;
    uint8_t initialFreq_mid;
    uint8_t initialFreq_lsb;

    // Save context
    // this part has been changed somewhat from the original semtech/upm driver
    // avoiding a call to the set frequency function would be easier here
    regPaConfigInitVal = readRegister( REG_PA_CONFIG );
    initialFreq_msb = readRegister(REG_FRF_MSB);
    initialFreq_mid = readRegister(REG_FRF_MID);
    initialFreq_lsb = readRegister(REG_FRF_LSB);

    // Cut the PA just in case, RFO output, power = -1 dBm
    writeRegister( REG_PA_CONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    writeRegister( REG_IMAGE_CAL, ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {}

    // Sets a Frequency in HF band
    setFrequency( 868000000 );

    // Launch Rx chain calibration for HF band
    writeRegister( REG_IMAGE_CAL, ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {}

    // Restore context
    writeRegister( REG_PA_CONFIG, regPaConfigInitVal );

    writeRegister(REG_FRF_MSB, initialFreq_msb);
    writeRegister(REG_FRF_MID, initialFreq_mid);
    writeRegister(REG_FRF_LSB, initialFreq_lsb);
}

// not sure of the use of this function but putting it in
bool SX1276IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh)
{
    int rssi = 0;

    SX1276SetModem(modem);

    setFrequency(freq);

    SX1276SetOpMode(MODE_RX_CONTINUOUS);

    upm_delay_us(1000);

    rssi = packetRssi(modem);

    SX1276sleep( );

    if( rssi > rssiThresh )
    {
        return false;
    }
    return true;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    writeRegister( REG_LR_IRQ_FLAGS_MASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( MODE_RX_CONTINUOUS );

    for( i = 0; i < 32; i++ )
    {
        upm_delay_us(1000);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )readRegister( REG_RSSI_WIDEBAND ) & 0x01 ) << i;
    }

    SX1276sleep( );

    return rnd;
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t SX1276GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    //while( 1 );
    printf("BW value not found\n");
    return 0;
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous ) {

    SX1276SetModem(modem);

    switch(modem){
        case MODEM_FSK:
            {
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.BandwidthAfc = bandwidthAfc;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.PayloadLen = payloadLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.RxContinuous = rxContinuous;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            writeRegister( REG_BITRATE_MSB, ( uint8_t )( datarate >> 8 ) );
            writeRegister( REG_BITRATE_LSB, ( uint8_t )( datarate & 0xFF ) );

            writeRegister( REG_RXBW, SX1276GetFskBandwidthRegValue( bandwidth ) );
            writeRegister( REG_AFCBW, SX1276GetFskBandwidthRegValue( bandwidthAfc ) );

            writeRegister( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            writeRegister( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                writeRegister( REG_FSK_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                writeRegister( REG_FSK_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

            writeRegister( REG_PACKETCONFIG1,
                         ( readRegister( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            writeRegister( REG_PACKETCONFIG2, ( readRegister( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;

        case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.PayloadLen = payloadLen;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            writeRegister( REG_MODEM_CONFIG_1,
                         ( readRegister( REG_MODEM_CONFIG_1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            writeRegister( REG_MODEM_CONFIG_2,
                         ( readRegister( REG_MODEM_CONFIG_2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            writeRegister( REG_MODEM_CONFIG_3,
                         ( readRegister( REG_MODEM_CONFIG_3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            writeRegister( REG_LR_SYMBTIMEOUT_LSB, ( uint8_t )( symbTimeout & 0xFF ) );

            writeRegister( REG_PREAMBLE_MSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            writeRegister( REG_PREAMBLE_LSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                writeRegister( REG_PAYLOAD_LENGTH, payloadLen );
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                writeRegister( REG_LR_PLLHOP, ( readRegister( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                writeRegister( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( SX1276.Settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                writeRegister( REG_LR_TEST36, 0x02 );
                writeRegister( REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                writeRegister( REG_LR_TEST36, 0x02 );
                writeRegister( REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                writeRegister( REG_LR_TEST36, 0x03 );
            }

            if( datarate == 6 )
            {
                writeRegister( REG_LR_DETECTOPTIMIZE,
                             ( readRegister( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                writeRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                writeRegister( REG_LR_DETECTOPTIMIZE,
                             ( readRegister( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                writeRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}


void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout ) {

    SX1276SetModem( modem );
    // not sure why this function is only called to set tx power
    // might have to set this explicitly for the rx side
    setTxPower(power, PA_OUTPUT_PA_BOOST_PIN);

    switch( modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.Fsk.Power = power;
            SX1276.Settings.Fsk.Fdev = fdev;
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            writeRegister( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            writeRegister( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            writeRegister( REG_BITRATE_MSB, ( uint8_t )( datarate >> 8 ) );
            writeRegister( REG_BITRATE_LSB, ( uint8_t )( datarate & 0xFF ) );

            writeRegister( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            writeRegister( REG_PREAMBLELSB, preambleLen & 0xFF );

            writeRegister( REG_PACKETCONFIG1,
                         ( readRegister( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            writeRegister( REG_PACKETCONFIG2, ( readRegister( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            SX1276.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                writeRegister( REG_LR_PLLHOP, ( readRegister( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                writeRegister( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            writeRegister( REG_MODEM_CONFIG_1,
                         ( readRegister( REG_MODEM_CONFIG_1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            writeRegister( REG_MODEM_CONFIG_2,
                         ( readRegister( REG_MODEM_CONFIG_2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            writeRegister( REG_MODEM_CONFIG_3,
                         ( readRegister( REG_MODEM_CONFIG_3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            writeRegister( REG_PREAMBLE_MSB, ( preambleLen >> 8 ) & 0x00FF );
            writeRegister( REG_PREAMBLE_LSB, preambleLen & 0xFF );
//
            if( datarate == 6 )
            {
                writeRegister( REG_LR_DETECTOPTIMIZE,
                             ( readRegister( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                writeRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                writeRegister( REG_LR_DETECTOPTIMIZE,
                             ( readRegister( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                writeRegister( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
            airTime = round( ( 8 * ( SX1276.Settings.Fsk.PreambleLen +
                                     ( ( readRegister( REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( SX1276.Settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( readRegister( REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( SX1276.Settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     SX1276.Settings.Fsk.Datarate ) * 1e3 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( SX1276.Settings.LoRa.Bandwidth )
            {
            //case 0: // 7.8 kHz
            //    bw = 78e2;
            //    break;
            //case 1: // 10.4 kHz
            //    bw = 104e2;
            //    break;
            //case 2: // 15.6 kHz
            //    bw = 156e2;
            //    break;
            //case 3: // 20.8 kHz
            //    bw = 208e2;
            //    break;
            //case 4: // 31.2 kHz
            //    bw = 312e2;
            //    break;
            //case 5: // 41.4 kHz
            //    bw = 414e2;
            //    break;
            //case 6: // 62.5 kHz
            //    bw = 625e2;
            //    break;
            case 7: // 125 kHz
                bw = 125e3;
                break;
            case 8: // 250 kHz
                bw = 250e3;
                break;
            case 9: // 500 kHz
                bw = 500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << SX1276.Settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( SX1276.Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * SX1276.Settings.LoRa.Datarate +
                                 28 + 16 * SX1276.Settings.LoRa.CrcOn -
                                 ( SX1276.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( SX1276.Settings.LoRa.Datarate -
                                 ( ( SX1276.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( SX1276.Settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor( tOnAir * 1e3 + 0.999 );
        }
        break;
    }
    return airTime;
}

// this will have a call to begin packet
void SX1276Send(uint8_t *buffer, uint8_t size)
{
    printf("coming into SX1276Send, modem: %d\n", SX1276.Settings.Modem);
    uint32_t txTimeout = 0;

    switch(SX1276.Settings.Modem)
    {
    case MODEM_FSK:
        {
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = size;

            if( SX1276.Settings.Fsk.FixLen == false )
            {
                //SX1276WriteFifo( ( uint8_t* )&size, 1 );
                write_buf(( uint8_t* )&size, 1);
            }
            else
            {
                writeRegister( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                SX1276.Settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy( RxTxBuffer, buffer, size );
                SX1276.Settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            write_buf( buffer, SX1276.Settings.FskPacketHandler.ChunkSize );
            SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
            txTimeout = SX1276.Settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                writeRegister(REG_LR_INVERTIQ, ((readRegister(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
                writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            SX1276.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            writeRegister( REG_PAYLOAD_LENGTH, 0 );

            // Full buffer used for Tx
            // make sure you get these right
            writeRegister(REG_FIFO_TX_BASE_ADDR, 0);  // not sure
            // this is usual
            writeRegister(REG_FIFO_ADDR_PTR, 0);

            // FIFO operations can not take place in Sleep mode
            if( ( readRegister( REG_OP_MODE ) & ~OP_MODE_MASK ) == MODE_SLEEP )
            {
                // setting to standby mode
                idle();
                upm_delay_us(1000);
            }
            //printf("prior to writing buffer\n");
            // Write payload buffer
            //SX1276WriteFifo( buffer, size );
            printf("size of the buffer being written: %d\n", size);
            write_buf(buffer, size);
            txTimeout = SX1276.Settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx(txTimeout);
}

// this function will have the endpacket code somewhere in it
void SX1276SetTx(uint32_t timeout)
{
    printf("coming into SX1276SetTx\n");
    //printf("final value of REG_PAYLOAD_LENGTH: %x\n", readRegister(REG_PAYLOAD_LENGTH));
    switch(SX1276.Settings.Modem)
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            writeRegister( REG_DIO_MAPPING_1, ( readRegister( REG_DIO_MAPPING_1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );

            writeRegister( REG_DIO_MAPPING_2, ( readRegister( REG_DIO_MAPPING_2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            SX1276.Settings.FskPacketHandler.FifoThresh = readRegister( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                writeRegister( REG_LR_IRQ_FLAGS_MASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                writeRegister(REG_DIO_MAPPING_1, (readRegister(REG_DIO_MAPPING_1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                writeRegister( REG_LR_IRQ_FLAGS_MASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                writeRegister(REG_DIO_MAPPING_1, (readRegister(REG_DIO_MAPPING_1) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    SX1276.Settings.State = RF_TX_RUNNING;
    printf("value of REG_IRQ_FLAGS very prior to clearing the interrupt: %x\n", readRegister(REG_IRQ_FLAGS));
    SX1276SetOpMode(MODE_TX);

    int count = 0;
    int irqFlags;
    // wait for TX done
    while(count < timeout) {
        irqFlags = readRegister(REG_IRQ_FLAGS);
        // condition here for tx done
        //if((irqFlags & RFLR_IRQFLAGS_TXDONE) == 0) {
        writeRegister(REG_IRQ_FLAGS, irqFlags);
        if(irqFlags & RFLR_IRQFLAGS_TXDONE) {
            printf("dio mapping 1: %x\n", readRegister(REG_DIO_MAPPING_1));
            printf("dio mapping 2: %x\n", readRegister(REG_DIO_MAPPING_2));
            // basically here we've got a tx done flag
            // clearing up the flags here
            //printf("value of REG_IRQ_FLAGS: %x\n", readRegister(REG_IRQ_FLAGS));
            printf("Transmission done\n");
            SX1276OnDio0Irq();
            //writeRegister(REG_IRQ_FLAGS, irqFlags);
            break;
        } else if (irqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK) {
            // fhss change channel interrupt if dio2 is set to 00
            SX1276OnDio2Irq();
        }
        count++;
        // 1 ms delay
        upm_delay_us(1000);
    }
    // this is where the timeout needs to be handled

    // clearing the register in case the interrupt occured
}

// comments to be added
void SX1276SetRx( uint32_t timeout )
{
    bool rxContinuous = false;

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = SX1276.Settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            writeRegister( REG_DIO_MAPPING_1, ( readRegister( REG_DIO_MAPPING_1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

            writeRegister( REG_DIO_MAPPING_2, ( readRegister( REG_DIO_MAPPING_2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            SX1276.Settings.FskPacketHandler.FifoThresh = readRegister( REG_FIFOTHRESH ) & 0x3F;

            writeRegister( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX1276.Settings.LoRa.Bandwidth < 9 )
            {
                writeRegister( REG_LR_DETECTOPTIMIZE, readRegister( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                writeRegister( REG_LR_TEST30, 0x00 );
                switch( SX1276.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    writeRegister( REG_LR_TEST2F, 0x48 );
                    setFrequency(SX1276.Settings.Channel + 7.81e3 );
                    break;
                case 1: // 10.4 kHz
                    writeRegister( REG_LR_TEST2F, 0x44 );
                    setFrequency(SX1276.Settings.Channel + 10.42e3 );
                    break;
                case 2: // 15.6 kHz
                    writeRegister( REG_LR_TEST2F, 0x44 );
                    setFrequency(SX1276.Settings.Channel + 15.62e3 );
                    break;
                case 3: // 20.8 kHz
                    writeRegister( REG_LR_TEST2F, 0x44 );
                    setFrequency(SX1276.Settings.Channel + 20.83e3 );
                    break;
                case 4: // 31.2 kHz
                    writeRegister( REG_LR_TEST2F, 0x44 );
                    setFrequency(SX1276.Settings.Channel + 31.25e3 );
                    break;
                case 5: // 41.4 kHz
                    writeRegister( REG_LR_TEST2F, 0x44 );
                    setFrequency(SX1276.Settings.Channel + 41.67e3 );
                    break;
                case 6: // 62.5 kHz
                    writeRegister( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    writeRegister( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    writeRegister( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                writeRegister( REG_LR_DETECTOPTIMIZE, readRegister( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX1276.Settings.LoRa.RxContinuous;

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                writeRegister( REG_LR_IRQ_FLAGS_MASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                writeRegister( REG_DIO_MAPPING_1, ( readRegister( REG_DIO_MAPPING_1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                writeRegister( REG_LR_IRQ_FLAGS_MASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                writeRegister( REG_DIO_MAPPING_1, ( readRegister( REG_DIO_MAPPING_1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            // again just like setTx be careful with these
            writeRegister( REG_FIFO_RX_BASE_ADDR, 0 );
            writeRegister( REG_FIFO_ADDR_PTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    SX1276.Settings.State = RF_RX_RUNNING;
    //if( timeout != 0 )
    //{
    //    TimerSetValue( &RxTimeoutTimer, timeout );
    //    TimerStart( &RxTimeoutTimer );
    //}

    if( SX1276.Settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( MODE_RX_CONTINUOUS );

        if( rxContinuous == false )
        {
// i wonder how this is even practically possible
// because they're explicitly setting the value to continuous right before the if check
// anyways not needed as fsk support not all that important rn
/*
            TimerSetValue( &RxTimeoutSyncWord, ceil( ( 8.0 * ( SX1276.Settings.Fsk.PreambleLen +
                                                             ( ( SX1276Read( REG_SYNCCONFIG ) &
                                                                ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                                1.0 ) + 10.0 ) /
                                                             ( double )SX1276.Settings.Fsk.Datarate ) * 1e3 ) + 4 );
            TimerStart( &RxTimeoutSyncWord );
*/
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX1276SetOpMode( MODE_RX_CONTINUOUS );
        }
        else
        {
            SX1276SetOpMode( MODE_RX_SINGLE );
        }
    }

    // here we listen for the packet
    int count = 0;
    int irqFlags;
    int packetLength = 0;
    // clearing the rx done flag
    rx_done = 0;
    while(count < timeout) {
        irqFlags = readRegister(REG_IRQ_FLAGS);
        //if ((irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) && (irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == 0) {
        if (irqFlags & RFLR_IRQFLAGS_RXDONE_MASK) {
            printf("interrupt occured: %x\n", irqFlags);
            //writeRegister(REG_IRQ_FLAGS, irqFlags);
            //SX1276._packetIndex = 0;
            // SX1276.Settings.LoRa.FixLen | if 0 then explicit
            //if (SX1276.Settings.LoRa.FixLen) {
            //    packetLength = readRegister(REG_PAYLOAD_LENGTH);
            //} else {
            //    packetLength = readRegister(REG_RX_NB_BYTES);
            //}
            //writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
            //idle();
            SX1276OnDio0Irq();
            break;
        } else if (irqFlags & RFLR_IRQFLAGS_RXTIMEOUT_MASK) {
            // rx timeout has occured - DIO1 is mapped to this if 00
            SX1276OnDio1Irq();
        } else if (irqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK) {
            // fhss change channel interrupt if dio2 is set to 00
            SX1276OnDio2Irq();
        }
        count++;
        upm_delay_us(1000);
    }

    if(!rx_done) {
        // call the timeout function
    }
    printf("timeout occured STATE: %d\n", SX1276.Settings.State);
    
/*
    if(packetLength) {
        printf("packet: \n");
        while(available()) {
            printf("%c", (char)SX1276read());
        }
        printf("\n");
    } else {
        printf("**** nothing Received ****\n");
        printf("**** Timeout Occured ****\n");
    }
*/
}

void SX1276StartCad(void)
{
    switch(SX1276.Settings.Modem)
    {
    case MODEM_FSK:
        {
            // nothing here \O/
        }
        break;
    case MODEM_LORA:
        {
            writeRegister ( REG_LR_IRQ_FLAGS_MASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            writeRegister( REG_DIO_MAPPING_1, ( readRegister( REG_DIO_MAPPING_1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            SX1276.Settings.State = RF_CAD;
            SX1276SetOpMode(MODE_CAD);
        }
        break;
    default:
        break;
    }
}

void SX1276SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    uint32_t timeout = ( uint32_t )( time * 1e3 );

    setFrequency( freq );

    // power = power
    // fdev = 0
    // bw = 0 | 125KHz
    // datarate = 4800
    // coderate = 0
    // preamble len = 5
    // fixlen = false
    // crcOn = false
    // freqHopOn = 0
    // hopPeriod = 0
    // iqInverted = 0
    // timeout = time*1000
    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    writeRegister( REG_PACKETCONFIG2, ( readRegister( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    writeRegister(REG_DIO_MAPPING_1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    writeRegister(REG_DIO_MAPPING_2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    //TimerSetValue( &TxTimeoutTimer, timeout );

    SX1276.Settings.State = RF_TX_RUNNING;
    //TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( MODE_TX );

    int count = 0;
    while(count < timeout) {
        // implement interrupt handling here
        count++;
        upm_delay_us(1000);
    }
}

void SX1276SetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    SX1276SetModem(modem);

    switch(modem)
    {
    case MODEM_FSK:
        if( SX1276.Settings.Fsk.FixLen == false )
        {
            writeRegister(REG_PAYLOADLENGTH, max);
        }
        break;
    case MODEM_LORA:
        writeRegister( REG_PAYLOAD_LENGTH, max );
        break;
    }
}

void SX1276SetPublicNetwork(bool enable)
{
    SX1276SetModem(MODEM_LORA);
    if(enable == true)
    {
        // Change LoRa modem SyncWord
        writeRegister(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        // Change LoRa modem SyncWord
        writeRegister( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

// this function will be called everytime timeout occurs
void SX1276OnTimeoutIrq(void)
{
    switch(SX1276.Settings.State)
    {
    case RF_RX_RUNNING:
        if(SX1276.Settings.Modem == MODEM_FSK)
        {
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            writeRegister( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            writeRegister( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( SX1276.Settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                writeRegister( REG_RXCONFIG, readRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                //TimerStart( &RxTimeoutSyncWord );
            }
            else
            {
                SX1276.Settings.State = RF_IDLE;
                //TimerStop( &RxTimeoutSyncWord );
            }
        }
// ** CALLBACKS HERE ** WILL HAVE TO BE IMPLMENTED WITH THE MAC LAYER **
/*
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
*/
        break;
    case RF_TX_RUNNING:
        SX1276.Settings.State = RF_IDLE;
// ** CALLBACKS HERE ** WILL HAVE TO BE IMPLMENTED WITH THE MAC LAYER **
/*
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
*/
        break;
    default:
        break;
    }
}

void SX1276OnDio0Irq(void)
{
    volatile uint8_t irqFlags = 0;

    switch(SX1276.Settings.State)
    {
        case RF_RX_RUNNING:
            switch(SX1276.Settings.Modem)
            {
            case MODEM_FSK:
                if( SX1276.Settings.Fsk.CrcOn == true )
                {
                    irqFlags = readRegister( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        writeRegister( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        writeRegister( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        //TimerStop( &RxTimeoutTimer );

                        if( SX1276.Settings.Fsk.RxContinuous == false )
                        {
                            //TimerStop( &RxTimeoutSyncWord );
                            SX1276.Settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            writeRegister( REG_RXCONFIG, readRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                            // need to handle this
                            //TimerStart( &RxTimeoutSyncWord );
                        }
                        printf("FSK: RX Error has occured\n");
// ** CALLBACKS HERE ** TO BE DETERMINED **
/*
                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
*/
                        SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                        SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                        SX1276.Settings.FskPacketHandler.NbBytes = 0;
                        SX1276.Settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

// **** READ FIFO IMPLEMENTATION FOR THIS DRIVER NEEDED ****

                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        //SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                        SX1276.Settings.FskPacketHandler.Size = readRegister(REG_FIFO);
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = readRegister( REG_PAYLOADLENGTH );
                    }

                    // experimental reading fifo here
                    SX1276._packetIndex = 0;
                    if(SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) {
                        while((SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) - SX1276._packetIndex) {
                            //printf("%c", (char)SX1276read());
                            RxTxBuffer[SX1276.Settings.FskPacketHandler.NbBytes + SX1276._packetIndex] = SX1276read();
                        }
                    }
                    //SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    //SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    // experimental reading fifo here
                    SX1276._packetIndex = 0;
                    if(SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) {
                        while((SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) - SX1276._packetIndex) {
                            RxTxBuffer[SX1276.Settings.FskPacketHandler.NbBytes + SX1276._packetIndex] = SX1276read();
                        }
                    }
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }

                //TimerStop( &RxTimeoutTimer );

                if( SX1276.Settings.Fsk.RxContinuous == false )
                {
                    SX1276.Settings.State = RF_IDLE;
                    //TimerStop( &RxTimeoutSyncWord );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    writeRegister( REG_RXCONFIG, readRegister( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                    //TimerStart( &RxTimeoutSyncWord );
                }
                printf("FSK: RX Done\n");
/*
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.FskPacketHandler.Size, SX1276.Settings.FskPacketHandler.RssiValue, 0 );
                }
*/
                SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                SX1276.Settings.FskPacketHandler.NbBytes = 0;
                SX1276.Settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    // Clear Irq
                    writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = readRegister( REG_IRQ_FLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        printf("LORA: RX Error has occured\n");
                        //TimerStop( &RxTimeoutTimer );

/*
                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
*/
                        break;
                    }

                    SX1276.Settings.LoRaPacketHandler.SnrValue = readRegister( REG_LR_PKTSNRVALUE );
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~SX1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = readRegister( REG_PKT_RSSI_VALUE );
                    if( snr < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                    }
                    else
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    if (SX1276.Settings.LoRa.FixLen) {
                        SX1276.Settings.LoRaPacketHandler.Size = readRegister(REG_PAYLOAD_LENGTH);
                    } else {
                        SX1276.Settings.LoRaPacketHandler.Size = readRegister(REG_RX_NB_BYTES);
                    }
                    printf("packet size retrieved: %d\n", SX1276.Settings.LoRaPacketHandler.Size);

                    // experimental addition, which actually seems to make things work
                    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

                    //SX1276.Settings.LoRaPacketHandler.Size = readRegister( REG_RX_NB_BYTES );
                    // read fifo to be implemented here with care
                    //SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );
                    SX1276._packetIndex = 0;
                    if(SX1276.Settings.LoRaPacketHandler.Size) {
                        while(SX1276.Settings.LoRaPacketHandler.Size - SX1276._packetIndex) {
                            RxTxBuffer[SX1276._packetIndex] = SX1276read();
                        }
                    }
                    // this is just for debug
                    int c = 0;
                    printf("Packet: ");
                    for(c=0; c<SX1276.Settings.LoRaPacketHandler.Size; c++)
                        //printf("%c", (char)RxTxBuffer[c]);
                        printf("%x, ", RxTxBuffer[c]);
                    printf("\n");

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    //TimerStop( &RxTimeoutTimer );
                    printf("LORA: RX Done\n");
                    rx_done = 1;
/*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
                    }
*/
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            //TimerStop( &TxTimeoutTimer );
            // TxDone interrupt
            switch( SX1276.Settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX1276.Settings.State = RF_IDLE;
                printf("LORA & FSK: TX Done\n");
/*
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
*/
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio1Irq(void)
{
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            switch(SX1276.Settings.Modem)
            {
            case MODEM_FSK:
                // FifoLevel interrupt
                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        //SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                        SX1276.Settings.FskPacketHandler.Size = readRegister(REG_FIFO);
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = readRegister( REG_PAYLOADLENGTH );
                    }
                }

                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.FifoThresh )
                {
                    // add fifo related code here carefully
                    //SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.FifoThresh );
                    SX1276._packetIndex = 0;
                    if(SX1276.Settings.FskPacketHandler.FifoThresh) {
                        while(SX1276.Settings.FskPacketHandler.FifoThresh - SX1276._packetIndex) {
                            RxTxBuffer[SX1276.Settings.FskPacketHandler.NbBytes + SX1276._packetIndex] = SX1276read();
                        }
                    }
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.FifoThresh;
                }
                else
                {
                    // fifo
                    //SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276._packetIndex = 0;
                    if(SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) {
                        while((SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes) - SX1276._packetIndex) {
                            RxTxBuffer[SX1276.Settings.FskPacketHandler.NbBytes + SX1276._packetIndex] = SX1276read();
                        }
                    }
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Sync time out
                //TimerStop( &RxTimeoutTimer );
                // Clear Irq
                writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX1276.Settings.State = RF_IDLE;
                printf("FIFO: DIO1: RX Timeout Occured\n");
/*
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
*/
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.ChunkSize )
                {
                    // write Buf here
                    //SX1276WriteFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.ChunkSize );
                    write_buf(( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.ChunkSize);
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    //SX1276WriteFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    write_buf(RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes);
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio2Irq(void)
{
    switch(SX1276.Settings.State)
    {
        case RF_RX_RUNNING:
            switch(SX1276.Settings.Modem)
            {
            case MODEM_FSK:
                // Checks if DIO4 is connected. If it is not PreambleDtected is set to true.
                // It is connected because it is being checked in rx running and tx running
                //if( SX1276.DIO4.port == NULL )
                //{
                    SX1276.Settings.FskPacketHandler.PreambleDetected = true;
                //}

                if( ( SX1276.Settings.FskPacketHandler.PreambleDetected == true ) && ( SX1276.Settings.FskPacketHandler.SyncWordDetected == false ) )
                {
                    //TimerStop( &RxTimeoutSyncWord );

                    SX1276.Settings.FskPacketHandler.SyncWordDetected = true;

                    SX1276.Settings.FskPacketHandler.RssiValue = -( readRegister( REG_RSSIVALUE ) >> 1 );

                    SX1276.Settings.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t )readRegister( REG_AFCMSB ) << 8 ) |
                                                                           ( uint16_t )readRegister( REG_AFCLSB ) ) *
                                                                           ( double )FREQ_STEP;
                    SX1276.Settings.FskPacketHandler.RxGain = ( readRegister( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( SX1276.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

// ** NOT IMPLEMENTED YET ** TBD **
/*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
*/
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch(SX1276.Settings.Modem)
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if(SX1276.Settings.LoRa.FreqHopOn == true)
                {
                    // Clear Irq
                    writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
// ** NOT IMPLEMENTED YET ** TBD **
/*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
*/
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio3Irq(void)
{
    switch(SX1276.Settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( readRegister( REG_IRQ_FLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );

// ** NOT IMPLEMENTED YET **
/*
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
*/
        }
        else
        {
            // Clear Irq
            writeRegister( REG_IRQ_FLAGS, RFLR_IRQFLAGS_CADDONE );
// ** NOT IMPLEMENTED YET **
/*
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
*/
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio4Irq(void)
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            if( SX1276.Settings.FskPacketHandler.PreambleDetected == false )
            {
                SX1276.Settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

uint8_t SX1276GetRxBufferlen() {
    return SX1276.Settings.LoRaPacketHandler.Size;
}

void SX1276GetRxBuffer(uint8_t* buf) {
    int c = 0;
    for (c = 0; c < SX1276.Settings.LoRaPacketHandler.Size; c++)
        buf[c] = RxTxBuffer[c];
}
