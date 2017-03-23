/*
 * Author: Jon Trulson <jtrulson@ics.com>
 * Contributions: Abhishek Malik <abhishek.malik@intel.com>
 * Copyright (c) 2017 Intel Corporation.
 *
 * Thanks to Semtech for their example code at:
 * https://github.com/Lora-net/LoRaMac-node
 * released under a modified BSD license, for many clues as to how to
 * initialize and operate this radio properly.
 * See src/sx1276/LICENSE.txt
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "sx1276.h"

#define PERIOD 1000
static const uint8_t m_writeMode = 0x80;
uint8_t buffer_arr[64] = {0};
uint8_t rx_buffer[FIFO_SIZE];
volatile int rx_RSSI;
volatile int rx_SNR;
volatile int rx_Len;

static RadioEvents_Callbacks *RadioEvents_c;
//static k_timer TxTimeoutTimer;

void sx1276_on_dio0_irq();
void sx1276_on_dio1_irq();
void sx1276_on_dio2_irq();
void sx1276_on_dio3_irq();
void sx1276_on_dio4_irq();
//void sx1276_on_dio5_irq(void *arg);
upm_result_t sx1276_cs_on(sx1276_context dev);
upm_result_t sx1276_cs_off(sx1276_context dev);

upm_result_t sx1276_dio0_qos(sx1276_context dev);
upm_result_t sx1276_dio1_qos(sx1276_context dev);
upm_result_t sx1276_dio2_qos(sx1276_context dev);
upm_result_t sx1276_dio3_qos(sx1276_context dev);

sx1276_context sx1276_init(int bus, int cs, int reset_pin, int dio0, int dio1,
                           int dio2, int dio3, int dio4, int dio5
#if defined(CONFIG_UPM_sx1276_mac)
, RadioEvents_Callbacks *events
#endif
)
{
    // make sure MRAA is initialized
    int mraa_rv;
    if((mraa_rv = mraa_init()) != MRAA_SUCCESS) {
        printf("%s: mraa_init failed (%d).\n", __FUNCTION__, mraa_rv);
        return NULL;
    }

#if defined(CONFIG_UPM_sx1276_mac)
    RadioEvents_c = events;
#endif
    //k_timer_init(&TxTimeoutTimer, );
#if 1
    // some initial setup
    typedef struct
    {
        RADIO_MODEM_T  Modem;
        uint8_t        Addr;
        uint8_t        Value;
    } radioRegisters_t;

    static const radioRegisters_t radioRegsInit[] = {
        { MODEM_FSK , COM_RegLna             , 0x23 },
        { MODEM_FSK , FSK_RegRxConfig        , 0x1E },
        { MODEM_FSK , FSK_RegRssiConfg       , 0xD2 },
        { MODEM_FSK , FSK_RegPreambleDetect  , 0xAA },
        { MODEM_FSK , FSK_RegOsc             , 0x07 },
        { MODEM_FSK , FSK_RegSyncConfig      , 0x12 },
        { MODEM_FSK , FSK_RegSyncValue1      , 0xC1 },
        { MODEM_FSK , FSK_RegSyncValue2      , 0x94 },
        { MODEM_FSK , FSK_RegSyncValue3      , 0xC1 },
        { MODEM_FSK , FSK_RegPacketConfig1   , 0xD8 },
        { MODEM_FSK , FSK_RegFifoThresh      , 0x8F },
        { MODEM_FSK , FSK_RegImageCal        , 0x02 },
        { MODEM_FSK , COM_RegDioMapping1     , 0x00 },
        { MODEM_FSK , COM_RegDioMapping2     , 0x30 },
        { MODEM_LORA, LOR_RegMaxPayloadLength, 0x40 }
    };
#endif

    sx1276_context dev = (sx1276_context) malloc(sizeof(struct _sx1276_context));
    if(!dev)
        return NULL;

    dev->settings = (m_settings) malloc(sizeof(struct _m_settings));
    if(!dev->settings) {
        printf("settings is nullllllllllll\n");
        return NULL;
    }

    radioLoRaSettings_t lora_settings = (radioLoRaSettings_t) malloc(sizeof(struct _radioLoRaSettings_t));
    if(!lora_settings) {
        printf("Null lora settings\n");
        return NULL;
    }
    dev->settings->loraSettings = lora_settings;

    if (!(dev->spi = mraa_spi_init(0))) {
        printf("sx1276: init failed\n");
        free(dev);
        return NULL;
    }
#if 1

    if(mraa_spi_mode(dev->spi, MRAA_SPI_MODE1) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set SPI mode\n");
        free(dev);
        return NULL;
    }
    if(mraa_spi_frequency(dev->spi, 4000000) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set higher frequency\n");
        free(dev);
        return NULL;
    }
#endif

    if(!(dev->gpio_cs = mraa_gpio_init(cs))) {
        printf("sx1276: unable to init SPI CS\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_dir(dev->gpio_cs, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        printf("sx1276: unable to set direction on SPI CS\n");
        free(dev);
        return NULL;
    }

    sx1276_cs_off(dev);

    if(!(dev->gpio_reset = mraa_gpio_init(reset_pin))) {
        printf("sx1276: unable to init chip reset\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_dir(dev->gpio_reset, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        printf("sx1276: unable to set direction on reset pin\n");
        free(dev);
        return NULL;
    }

    upm_delay_ms(10);
    //k_busy_wait(10000);
    int_flag=0;

#if 0
    if(mraa_gpio_write(dev->gpio_reset, 1) != MRAA_SUCCESS) { 
        printf("unable to write the reset pin\n");
    }
#endif

    if(!(dev->gpio_dio0 = mraa_gpio_init(dio0)) || !(dev->gpio_dio1 = mraa_gpio_init(dio1))
       || !(dev->gpio_dio2 = mraa_gpio_init(dio2)) || !(dev->gpio_dio3 = mraa_gpio_init(dio3))
       || !(dev->gpio_dio4 = mraa_gpio_init(dio4)) || !(dev->gpio_dio5 = mraa_gpio_init(dio5))) {
        printf("sx1276: unable to init an interrupt pin on the LORA\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_dir(dev->gpio_dio0, MRAA_GPIO_IN) != MRAA_SUCCESS ||
       mraa_gpio_dir(dev->gpio_dio1, MRAA_GPIO_IN) != MRAA_SUCCESS ||
       mraa_gpio_dir(dev->gpio_dio2, MRAA_GPIO_IN) != MRAA_SUCCESS ||
       mraa_gpio_dir(dev->gpio_dio3, MRAA_GPIO_IN) != MRAA_SUCCESS ||
       mraa_gpio_dir(dev->gpio_dio4, MRAA_GPIO_IN) != MRAA_SUCCESS ||
       mraa_gpio_dir(dev->gpio_dio5, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        printf("sx1276: unable to set direction on one of the interrupt pins\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_isr(dev->gpio_dio0, MRAA_GPIO_EDGE_RISING, sx1276_on_dio0_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio0 ISR\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_isr(dev->gpio_dio1, MRAA_GPIO_EDGE_RISING, sx1276_on_dio1_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio1 ISR\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_isr(dev->gpio_dio2, MRAA_GPIO_EDGE_RISING, sx1276_on_dio2_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio2 ISR\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_isr(dev->gpio_dio3, MRAA_GPIO_EDGE_RISING, sx1276_on_dio3_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio3 ISR\n");
        free(dev);
        return NULL;
    }

    if(mraa_gpio_isr(dev->gpio_dio4, MRAA_GPIO_EDGE_RISING, sx1276_on_dio4_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio4 ISR\n");
        free(dev);
        return NULL;
    }
#if 0
    if(mraa_gpio_isr(dev->gpio_dio5, MRAA_GPIO_EDGE_RISING, sx1276_on_dio5_irq, NULL) != MRAA_SUCCESS) {
        printf("sx1276: Unable to set up the dio5 ISR\n");
        free(dev);
        return NULL;
    }
#endif

    r_event = REVENT_DONE;
    dev->settings->state = STATE_IDLE;
    memset(rx_buffer, 0, FIFO_SIZE);
    rx_SNR = 0;
    rx_RSSI = 0;

    uint8_t chip_id;

#if 1
    if(sx1276_get_chip_version(dev, &chip_id) != UPM_SUCCESS) {
        printf("sx1276: Unable to read the chip ID\n");
        return NULL;
    }
    if (chip_id != chipRevision) { 
        printf("sx1276: Incorrect chip version, expected: %x, got: %x\n", chipRevision, chip_id);
        return NULL;
    }

    if(sx1276_reset(dev) != UPM_SUCCESS) {
        return NULL;
    }

    
#endif
    printf("chip id was returned: %x\n", chip_id);

    if(sx1276_rx_chain_calibration(dev) != UPM_SUCCESS) {
        printf("Unable to do rx chain calibration\n");
        return NULL;
    }

    if(sx1276_set_op_mode(dev, MODE_Sleep) != UPM_SUCCESS) {
        printf("sx1276: Unable to set mode to sleep\n");
        return NULL;
    }

    for (size_t i = 0; i < sizeof(radioRegsInit) / sizeof(radioRegisters_t); i++ ) {
        sx1276_set_modem(dev, radioRegsInit[i].Modem);
        sx1276_write_reg(dev, radioRegsInit[i].Addr, radioRegsInit[i].Value);
    }

    sx1276_set_modem(dev, MODEM_FSK);
    dev->settings->state = STATE_IDLE;

    return dev;
}

upm_result_t sx1276_get_chip_version(sx1276_context dev, uint8_t* chip_id) {
    uint8_t ret = sx1276_read_reg(dev, COM_RegVersion);
    if(ret == -1) {
        return UPM_ERROR_OPERATION_FAILED;
    }
    *chip_id = ret;

    return UPM_SUCCESS;
}

uint8_t sx1276_read_reg(sx1276_context dev, uint8_t reg) {
    uint8_t tx_buf[2] = { reg & 0x7f, 0 };
    uint8_t rx_buf[2];

    sx1276_cs_on(dev);
    if (mraa_spi_transfer_buf(dev->spi, tx_buf, rx_buf, 2) != MRAA_SUCCESS) {
        printf("Unable to transfer data over the SPI bus\n");
        printf("SPI operation failed\n");
        sx1276_cs_off(dev);
        return -1;
    }
    sx1276_cs_off(dev);

    //*reg_val = rx_buf[1];
    return rx_buf[1];
}

upm_result_t sx1276_write_reg(sx1276_context dev, uint8_t reg, uint8_t val) {
    uint8_t pkt[2] = {(uint8_t)(reg | m_writeMode), val};

    sx1276_cs_on(dev);
    if (mraa_spi_transfer_buf(dev->spi, pkt, NULL, 2) != MRAA_SUCCESS) {
        printf("Unable to transfer data over the SPI bus\n");
        printf("SPI operation failed\n");
        sx1276_cs_off(dev);
        return UPM_ERROR_OPERATION_FAILED;
    }
    sx1276_cs_off(dev);

    return UPM_SUCCESS;
}

upm_result_t sx1276_reset(sx1276_context dev){

    if(mraa_gpio_write(dev->gpio_reset, 0) != MRAA_SUCCESS) { 
        printf("unable to write the reset pin\n");
    }
    if(mraa_gpio_write(dev->gpio_reset, 1) != MRAA_SUCCESS) { 
        printf("unable to write the reset pin\n");
    }

    return UPM_SUCCESS;
}

upm_result_t sx1276_rx_chain_calibration(sx1276_context dev){
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;
    uint8_t reg;

    // this function should only be called in init() (after reset()), as
    // the device is configured for FSK mode, LF at that time.

    // Save context
    regPaConfigInitVal = sx1276_read_reg(dev, COM_RegPaConfig);
    if(regPaConfigInitVal == -1) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    initialFreq = (uint32_t) ( ((double)
                               (((uint32_t)sx1276_read_reg(dev, COM_RegFrfMsb) << 16) |
                               ((uint32_t)sx1276_read_reg(dev, COM_RegFrfMid) << 8) |
                               ((uint32_t)sx1276_read_reg(dev, COM_RegFrfLsb)) ) )
                               * FXOSC_STEP);

    // Cut the PA just in case, RFO output, power = -1 dBm
    if(sx1276_write_reg(dev, COM_RegPaConfig, 0x00) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    // Launch Rx chain calibration for LF band
    reg = sx1276_read_reg(dev, FSK_RegImageCal);
    if(reg == -1) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    if(sx1276_write_reg(dev, FSK_RegImageCal, reg | IMAGECAL_ImageCalStart) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    // spin until complete
    while(sx1276_read_reg(dev, FSK_RegImageCal) & IMAGECAL_ImageCalRunning) {
        //k_busy_wait(10);
        upm_delay_ms(1);
    }

    //  cerr << __FUNCTION__ << ": Imagecal LF complete" << endl;
    // Set a Frequency in HF band
#if 1
    if(sx1276_set_channel(dev, 868000000) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    reg = sx1276_read_reg(dev, FSK_RegImageCal);
    if(reg == -1) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    if(sx1276_write_reg(dev, FSK_RegImageCal, reg | IMAGECAL_ImageCalStart) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    // spin until complete
    while(sx1276_read_reg(dev, FSK_RegImageCal) & IMAGECAL_ImageCalRunning) {
        //k_busy_wait(10);
        upm_delay_ms(1);
    }

    //  cerr << __FUNCTION__ << ": Imagecal LF complete" << endl;
#endif

    // Restore context
    //sx1276_write_reg(dev, COM_RegPaConfig, regPaConfigInitVal);
    if(sx1276_write_reg(dev, COM_RegPaConfig, regPaConfigInitVal) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    if(sx1276_set_channel(dev, initialFreq) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }
    //setChannel(initialFreq);
    return UPM_SUCCESS;
}

upm_result_t sx1276_set_channel(sx1276_context dev, uint32_t freq) {
    dev->settings->channel = freq;

    freq = ( uint32_t )( ( double )freq / FXOSC_STEP );
#if 1
    if(sx1276_write_reg(dev, COM_RegFrfMsb, ( uint8_t )( ( freq >> 16 ) & 0xff ) ) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }
    if(sx1276_write_reg(dev, COM_RegFrfMid, ( uint8_t )( ( freq >> 8 ) & 0xff ) ) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }
    if(sx1276_write_reg(dev, COM_RegFrfLsb, ( uint8_t )( freq & 0xff ) ) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }
#endif
    return UPM_SUCCESS;
}

upm_result_t sx1276_set_modem(sx1276_context dev, RADIO_MODEM_T modem) {
    if(dev->settings->modem == modem) {
        printf("modem value is the same, returning\n");
        return UPM_SUCCESS;
    }
    dev->settings->modem = modem;

    uint8_t reg = 0;
    switch (dev->settings->modem) {
        default:
        case MODEM_FSK:
        break;
        case MODEM_LORA: {
            if(sx1276_set_op_mode(dev, MODE_Sleep) != UPM_SUCCESS) {
                printf("sx1276: unable to set op mode in set mode\n");
            }
            // turn lora on
            reg = (sx1276_read_reg(dev, COM_RegOpMode) | OPMODE_LongRangeMode);
            if(reg == -1) {
                printf("sx1276: bogus value\n");
            }
            sx1276_write_reg(dev, COM_RegOpMode, reg);

            sx1276_write_reg(dev, COM_RegDioMapping1, 0x00);
            sx1276_write_reg(dev, COM_RegDioMapping2, 0x00);
        }
        break;
    }
    //printf("modem value: %d\n", dev->settings->modem);
    return UPM_SUCCESS;
}

upm_result_t sx1276_set_tx_config(sx1276_context dev, RADIO_MODEM_T modem, int8_t power,
                                  uint32_t fdev, uint32_t bandwidth, uint32_t datarate,
                                  uint8_t coderate, uint16_t preamble_len,
                                  bool fix_len, bool crc_on, bool freq_hop_on,
                                  uint8_t hop_period, bool iq_inverted) {
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    sx1276_set_modem(dev, modem);

    paConfig = sx1276_read_reg(dev, COM_RegPaConfig);
    paDac = sx1276_read_reg(dev, COM_RegPaDac);

    uint8_t paSelect = 0x00; // default, +14dBm
    if (dev->settings->channel < RF_MID_BAND_THRESH)
    paSelect = PACONFIG_PaSelect; // PA_BOOST, +20dBm

    paConfig &= ~PACONFIG_PaSelect;
    paConfig |= paSelect;
    paConfig &= ~(_PACONFIG_MaxPower_MASK << _PACONFIG_MaxPower_SHIFT);
    paConfig |= (7 << _PACONFIG_MaxPower_SHIFT); // PACONFIG_MaxPower = 7

    if ((paConfig & PACONFIG_PaSelect)) {
        if (power > 17) {
            paDac &= ~(_PADAC_PaDac_MASK << _PADAC_PaDac_SHIFT);
            paDac |= (PADAC_BOOST << _PADAC_PaDac_SHIFT);
        }
        else {
            paDac &= ~(_PADAC_PaDac_MASK << _PADAC_PaDac_SHIFT);
            paDac |= (PADAC_DEFAULT << _PADAC_PaDac_SHIFT);
        }
        if ((paDac & PADAC_BOOST) == PADAC_BOOST) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }
            paConfig = ~(_PACONFIG_OutputPower_MASK & _PACONFIG_OutputPower_SHIFT);
            paConfig |= ( ((uint8_t)(power - 5) & 
                         _PACONFIG_OutputPower_MASK) << 
                        _PACONFIG_OutputPower_SHIFT );
        } else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }
          
            paConfig = ~(_PACONFIG_OutputPower_MASK & _PACONFIG_OutputPower_SHIFT);
            paConfig |= ( ((uint8_t)(power - 2) & 
                         _PACONFIG_OutputPower_MASK) << 
                        _PACONFIG_OutputPower_SHIFT );
        }
    } else {
        if (power < -1) {
            power = -1;
        }
        if (power > 14) {
            power = 14;
        }

        paConfig = ~(_PACONFIG_OutputPower_MASK & _PACONFIG_OutputPower_SHIFT);
        paConfig |= ( ((uint8_t)(power + 1) & 
                    _PACONFIG_OutputPower_MASK) << 
                    _PACONFIG_OutputPower_SHIFT );
    }

    if(sx1276_write_reg(dev, COM_RegPaConfig, paConfig) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }
    if(sx1276_write_reg(dev, COM_RegPaDac, paDac) != UPM_SUCCESS) {
        return UPM_ERROR_OPERATION_FAILED;
    }

    uint8_t reg;

    switch (modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            dev->settings->loraSettings->Power = power;

            switch (bandwidth) {
            case 125000:
                bandwidth = BW_125;
            break;

            case 250000:
                bandwidth = BW_250;
            break;

            case 500000:
                bandwidth = BW_500;
            break;

            default:
                printf("sx1276: LORA bandwidth must be 125000, 250000 or 500000\n");
            }

            dev->settings->loraSettings->Bandwidth = bandwidth;
            dev->settings->loraSettings->Datarate = datarate;
            dev->settings->loraSettings->Coderate = coderate;
            dev->settings->loraSettings->PreambleLen = preamble_len;
            dev->settings->loraSettings->FixLen = fix_len;
            dev->settings->loraSettings->FreqHopOn = freq_hop_on;
            dev->settings->loraSettings->HopPeriod = hop_period;
            dev->settings->loraSettings->CrcOn = crc_on;
            dev->settings->loraSettings->IqInverted = iq_inverted;

            // datarate is really SPREADINGFACTOR_* for LoRa
            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }
        
            if ( ((bandwidth == BW_125) && ((datarate == 11) || 
                                        (datarate == 12))) ||
             ((bandwidth == BW_250) && (datarate == 12)) ) {
                dev->settings->loraSettings->LowDatarateOptimize = true;
            } else {
                dev->settings->loraSettings->LowDatarateOptimize = false;
            }

            // datasheet says this is only valid in FSK mode, but Semtech
            // code indicates it is only available in LORA mode... So
            // which is it?

            // Lets assume for now that the code is correct, as there
            // is a HopPeriod register for LoRa, and no such registers
            // exist for FSK.
            if (dev->settings->loraSettings->FreqHopOn == true) {
                reg = sx1276_read_reg(dev, LOR_RegPllHop);
                reg &= ~PLLHOP_FastHopOn;
                reg |= PLLHOP_FastHopOn;
                sx1276_write_reg(dev, LOR_RegPllHop, reg);

                sx1276_write_reg(dev, LOR_RegHopPeriod, dev->settings->loraSettings->HopPeriod);
            } else {
                reg = sx1276_read_reg(dev, LOR_RegPllHop);
                reg &= ~PLLHOP_FastHopOn;
                sx1276_write_reg(dev, LOR_RegPllHop, reg);
            }

            reg = sx1276_read_reg(dev, LOR_RegModemConfig1);

            reg &= ~((_MODEMCONFIG1_CodingRate_MASK << 
                   _MODEMCONFIG1_CodingRate_SHIFT) |
                   (_MODEMCONFIG1_Bw_MASK << _MODEMCONFIG1_Bw_SHIFT) |
                   MODEMCONFIG1_ImplicitHeaderModeOn);

            if (fix_len)
                reg |= MODEMCONFIG1_ImplicitHeaderModeOn;

            reg |= ((bandwidth & _MODEMCONFIG1_Bw_MASK) << _MODEMCONFIG1_Bw_SHIFT);
            reg |= ((coderate & _MODEMCONFIG1_CodingRate_MASK) << 
                   _MODEMCONFIG1_CodingRate_SHIFT);

            sx1276_write_reg(dev, LOR_RegModemConfig1, reg);

            reg = sx1276_read_reg(dev, LOR_RegModemConfig2);

            reg &= ~((_MODEMCONFIG2_SpreadingFactor_MASK << 
                   _MODEMCONFIG2_SpreadingFactor_SHIFT) |
                   MODEMCONFIG2_RxPayloadCrcOn);

            if (crc_on)
                reg |= MODEMCONFIG2_RxPayloadCrcOn;

            reg |= ((datarate & _MODEMCONFIG2_SpreadingFactor_MASK) << 
                   _MODEMCONFIG2_SpreadingFactor_SHIFT);

            sx1276_write_reg(dev, LOR_RegModemConfig2, reg);

            reg = sx1276_read_reg(dev, LOR_RegModemConfig3);

            reg &= ~MODEMCONFIG3_LowDataRateOptimize;

            if (dev->settings->loraSettings->LowDatarateOptimize)
                reg |= MODEMCONFIG3_LowDataRateOptimize;

            sx1276_write_reg(dev, LOR_RegModemConfig3, reg);

            sx1276_write_reg(dev, LOR_RegPreambleMsb, (uint8_t)((preamble_len >> 8) & 0xff));
            sx1276_write_reg(dev, LOR_RegPreambleLsb, (uint8_t)(preamble_len & 0xff));

            // datarate is SPREADINGFACTOR_*
            if (datarate == 6) {
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg &= ~(_DETECTOPTIMIZE_DetectionOptimize_MASK << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                reg |= (DETECTIONOPTIMIZE_SF6 << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);

                // see page 27 in the datasheet
                sx1276_write_reg(dev, LOR_RegDetectionThreshold, LOR_DetectionThreshold_SF6);
            } else {
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg &= ~(_DETECTOPTIMIZE_DetectionOptimize_MASK << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                reg |= (DETECTIONOPTIMIZE_SF7_SF12 << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);

                // see page 27 in the datasheet
                sx1276_write_reg(dev, LOR_RegDetectionThreshold, 
                         LOR_DetectionThreshold_SF7_SF12);
            }
        }
        break;
    }

    return UPM_SUCCESS;
}

upm_result_t sx1276_set_rx_config(sx1276_context dev, RADIO_MODEM_T modem,
                                  uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                                  uint32_t bandwidth_afc, uint16_t preamble_len,
                                  uint16_t symb_time_out, bool fix_len,
                                  uint8_t pay_load_len, bool crc_on,
                                  bool freq_hop_on, uint8_t hop_period,
                                  bool iq_inverted, bool rx_continuous) {
    sx1276_set_modem(dev, modem);

    uint8_t reg;

    switch (modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            // convert the supplied (legal) LORA bandwidths into something
            // the chip can handle.
            switch (bandwidth) {
                case 125000:
                    bandwidth = BW_125;
                break;
            
                case 250000:
                    bandwidth = BW_250;
                break;
            
                case 500000:
                    bandwidth = BW_500;
                break;
            
                default:
                printf("sx1276: LORA bandwidth must be 125000, 250000 or 500000\n");
            }

            dev->settings->loraSettings->Bandwidth = bandwidth;
            dev->settings->loraSettings->Datarate = datarate;
            dev->settings->loraSettings->Coderate = coderate;
            dev->settings->loraSettings->FixLen = fix_len;
            dev->settings->loraSettings->PayloadLen = pay_load_len;
            dev->settings->loraSettings->CrcOn = crc_on;
            dev->settings->loraSettings->FreqHopOn = freq_hop_on;
            dev->settings->loraSettings->HopPeriod = hop_period;
            dev->settings->loraSettings->IqInverted = iq_inverted;
            dev->settings->loraSettings->RxContinuous = rx_continuous;

            // datarate is really LORA SPREADING_FACTOR_*
            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }
        
            if ( ((bandwidth == BW_125) && ((datarate == 11) || 
                                        (datarate == 12))) ||
               ((bandwidth == BW_250) && (datarate == 12)) ) {
                dev->settings->loraSettings->LowDatarateOptimize = true;
            } else {
                dev->settings->loraSettings->LowDatarateOptimize = false;
            }

            reg = sx1276_read_reg(dev, LOR_RegModemConfig1);
            reg &= ~((_MODEMCONFIG1_CodingRate_MASK << 
                   _MODEMCONFIG1_CodingRate_SHIFT) |
                   (_MODEMCONFIG1_Bw_MASK << _MODEMCONFIG1_Bw_SHIFT) |
                   MODEMCONFIG1_ImplicitHeaderModeOn);

            if (fix_len)
                reg |= MODEMCONFIG1_ImplicitHeaderModeOn;

            reg |= ((bandwidth & _MODEMCONFIG1_Bw_MASK) << _MODEMCONFIG1_Bw_SHIFT);
            reg |= ((coderate & _MODEMCONFIG1_CodingRate_MASK) << 
                   _MODEMCONFIG1_CodingRate_SHIFT);

            sx1276_write_reg(dev, LOR_RegModemConfig1, reg);

            reg = sx1276_read_reg(dev, LOR_RegModemConfig2);
            reg &= ~((_MODEMCONFIG2_SpreadingFactor_MASK << 
                   _MODEMCONFIG2_SpreadingFactor_SHIFT) |
                   MODEMCONFIG2_RxPayloadCrcOn |
                   (_MODEMCONFIG2_SymbTimeoutMsb_MASK <<
                   _MODEMCONFIG2_SymbTimeoutMsb_SHIFT));

            if (crc_on)
                reg |= MODEMCONFIG2_RxPayloadCrcOn;

            reg |= ((datarate & _MODEMCONFIG2_SpreadingFactor_MASK) << 
                   _MODEMCONFIG2_SpreadingFactor_SHIFT);

            // mask symbTimeOut (MSB) for safety
            reg |= ( ((symb_time_out >> 8) & _MODEMCONFIG2_SymbTimeoutMsb_MASK) << 
                   _MODEMCONFIG2_SymbTimeoutMsb_SHIFT);

            sx1276_write_reg(dev, LOR_RegModemConfig2, reg);

            reg = sx1276_read_reg(dev, LOR_RegModemConfig3);
        
            reg &= ~MODEMCONFIG3_LowDataRateOptimize;

            if (dev->settings->loraSettings->LowDatarateOptimize)
                reg |= MODEMCONFIG3_LowDataRateOptimize;

            sx1276_write_reg(dev, LOR_RegModemConfig3, reg);

            sx1276_write_reg(dev, LOR_RegSymbTimeoutLsb, (uint8_t)(symb_time_out & 0xff));

            sx1276_write_reg(dev, LOR_RegPreambleMsb, (uint8_t)((preamble_len >> 8) & 0xff));
            sx1276_write_reg(dev, LOR_RegPreambleLsb, (uint8_t)(preamble_len & 0xff));

            if (fix_len == 1)
                sx1276_write_reg(dev, LOR_RegPayloadLength, pay_load_len);

            // The datasheet says this is only valid in FSK mode, but
            // Semtech code indicates it is only available in LORA
            // mode... So which is it?
        
            // Lets assume for now that the code is correct, as there
            // is a HopPeriod register for LoRa, and no such registers
            // exist for FSK.
            if (dev->settings->loraSettings->FreqHopOn) {
                reg = sx1276_read_reg(dev, LOR_RegPllHop);
                reg &= ~PLLHOP_FastHopOn;
                reg |= PLLHOP_FastHopOn;
                sx1276_write_reg(dev, LOR_RegPllHop, reg);

                sx1276_write_reg(dev, LOR_RegHopPeriod, dev->settings->loraSettings->HopPeriod);
            } else {
                reg = sx1276_read_reg(dev, LOR_RegPllHop);
                reg &= ~PLLHOP_FastHopOn;
                sx1276_write_reg(dev, LOR_RegPllHop, reg);
            }

            // errata checks - writing magic numbers into undocumented,
            // reserved registers :) The Semtech code was broken in this
            // logic.
            if ( (bandwidth == BW_500) && 
               (dev->settings->channel > RF_MID_BAND_THRESH) ) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz
                // Bandwidth (HF)
                sx1276_write_reg(dev, LOR_Reserved36, 0x02);
                sx1276_write_reg(dev, LOR_Reserved3a, 0x64);
            } else if (bandwidth == BW_500 && 
                      (dev->settings->channel >= 410000000)) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz
                // Bandwidth (LF above 410Mhz)
                sx1276_write_reg(dev, LOR_Reserved36, 0x02);
                sx1276_write_reg(dev, LOR_Reserved3a, 0x7f);
            } else {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz
                // Bandwidth (everything else)
                sx1276_write_reg(dev, LOR_Reserved36, 0x03);
            }

            if (datarate == 6) {
                // datarate == SPREADINGFACTOR_64
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg &= ~(_DETECTOPTIMIZE_DetectionOptimize_MASK << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                reg |= (DETECTIONOPTIMIZE_SF6 << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);

                // see page 27 in the datasheet
                sx1276_write_reg(dev, LOR_RegDetectionThreshold, LOR_DetectionThreshold_SF6);
            } else {
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg &= ~(_DETECTOPTIMIZE_DetectionOptimize_MASK << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                reg |= (DETECTIONOPTIMIZE_SF7_SF12 << 
                       _DETECTOPTIMIZE_DetectionOptimize_SHIFT);

                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);

                // see page 27 in the datasheet
                sx1276_write_reg(dev, LOR_RegDetectionThreshold, 
                         LOR_DetectionThreshold_SF7_SF12);
             }
        }
    break;
    }

    return UPM_SUCCESS;
}

RADIO_EVENT_T sx1276_send_str(sx1276_context dev, char* buffer, int timeout) {
    //if (buffer.size() > (FIFO_SIZE - 1))
    int size = strlen(buffer);
    if (size > (FIFO_SIZE - 1))
        printf("sx1276: buffer size must be less than 256\n");

    // for LORA/FSK modem, there seems to be a 64 byte requirement,
    // (LOR_RegRxNbBytes on the receiver) never seems to be anything
    // other than 64. Same seems to go for the FSK modem.  So, if the
    // packet is less than 64, pad it out to 64 bytes.  This requires
    // investigation.
    //while (buffer.size() < 64)
    //    buffer.push_back(0);
    // using a uint8_t array initialized to zero here
    // and then using memcpy to copy over the string
    RADIO_EVENT_T event;
    if(size < 64) {
        memset(buffer_arr, 0, 64);
        memcpy(buffer_arr, buffer, size);
        event = sx1276_send(dev, buffer_arr, 64, timeout);
    } else {
        event = sx1276_send(dev, buffer, size, timeout);
    }

    return event;
}

RADIO_EVENT_T sx1276_send(sx1276_context dev, uint8_t* buffer, uint8_t size, int timeout) {
    switch (dev->settings->modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            if (dev->settings->loraSettings->IqInverted == true) {
                uint8_t reg = sx1276_read_reg(dev, LOR_RegInvertIQ);

                reg &= ~(INVERTIQ_InvertIQTxOff | INVERTIQ_InvertIQRx);
                sx1276_write_reg(dev, LOR_RegInvertIQ, reg);

                // warning, hardcoded undocumented magic number into
                // undocumented register
                sx1276_write_reg(dev, LOR_RegInvertIQ2, 0x19);
            } else {
                uint8_t reg = sx1276_read_reg(dev, LOR_RegInvertIQ);
                reg &= ~(INVERTIQ_InvertIQTxOff | INVERTIQ_InvertIQRx);
                reg |= INVERTIQ_InvertIQTxOff; // 'active' off.
                sx1276_write_reg(dev, LOR_RegInvertIQ, reg);

                // warning, hardcoded undocumented magic number into
                // undocumented register
                sx1276_write_reg(dev, LOR_RegInvertIQ2, 0x1d);
            }

            dev->settings->loraPacketHandler->Size = size;
            //        cerr << "PAYLOAD SIZE " << (int)size << endl;

            // Initializes the payload size
            sx1276_write_reg(dev, LOR_RegPayloadLength, size);

            // Full buffer used for Tx
            sx1276_write_reg(dev, LOR_RegFifoTxBaseAddr, 0);
            sx1276_write_reg(dev, LOR_RegFifoAddrPtr, 0 );

            // FIFO operations can not take place in Sleep mode
            if ((sx1276_read_reg(dev, COM_RegOpMode) & _OPMODE_Mode_MASK) == MODE_Sleep) {
                sx1276_set_stand_by(dev);
printf("coming here\n");
                upm_delay_ms(1);
                //k_busy_wait(1000);
                //usleep(1000); // 1ms
            }

            // Write payload buffer
            if(sx1276_write_fifo(dev, buffer, size) != UPM_SUCCESS) {
                printf("unable to write payload buffer\n");
                //return UPM_ERROR_OPERATION_FAILED;
            }
        }
        break;
    }

    //RADIO_EVENT_T r_event;
    //if(sx1276_set_tx(dev, timeout) != UPM_SUCCESS) {
        //printf("sx1276: some error in the set tx function\n");
        //return UPM_ERROR_OPERATION_FAILED;
    //}
    //*radio_event = r_event;
    return sx1276_set_tx(dev, timeout);
}

upm_result_t sx1276_write_fifo(sx1276_context dev, uint8_t* buffer, int len) {
    // can't write more than 256 bytes
    if (len > FIFO_SIZE) {
        printf("sx1276: cannot write more than 256 bytes to FIFO\n");
        return UPM_ERROR_OPERATION_FAILED;
    }

    uint8_t pkt = (0 | m_writeMode);

    sx1276_cs_on(dev);
    if (mraa_spi_transfer_buf(dev->spi, &pkt, NULL, 1) != MRAA_SUCCESS) {
        printf("sx1276: Spi.transfer(NULL) failed\n");
        sx1276_cs_off(dev);
        return UPM_ERROR_OPERATION_FAILED;
    }

//printf("%d\t%d\t%d\t%d\t%d\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    if (mraa_spi_transfer_buf(dev->spi, buffer, NULL, len) != MRAA_SUCCESS){
        printf("sx1276: Spi.transfer(buf) failed");
        sx1276_cs_off(dev);
        return UPM_ERROR_OPERATION_FAILED;
    }
    sx1276_cs_off(dev);

    return UPM_SUCCESS;
}

upm_result_t sx1276_set_stand_by(sx1276_context dev) {
    sx1276_set_op_mode(dev, MODE_Standby);
    dev->settings->state = STATE_IDLE;
    return UPM_SUCCESS;
}

upm_result_t sx1276_set_op_mode(sx1276_context dev, MODE_T op_mode){
    static uint8_t opModePrev = MODE_Standby;

    if(op_mode != opModePrev) {
        opModePrev = op_mode;

        uint8_t reg = sx1276_read_reg(dev, COM_RegOpMode) & 
                      ~(_OPMODE_Mode_MASK << _OPMODE_Mode_SHIFT);

        sx1276_write_reg(dev, COM_RegOpMode, (reg | (op_mode << _OPMODE_Mode_SHIFT)) );
    }

    return UPM_SUCCESS;
}

RADIO_EVENT_T sx1276_set_tx(sx1276_context dev, int timeout){
printf("coming into set tx\n");
    uint8_t reg = 0;
    switch (dev->settings->modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            if (dev->settings->loraSettings->FreqHopOn == true ) {
                // mask out all except TxDone and FhssChangeChannel
                sx1276_write_reg(dev, LOR_RegIrqFlagsMask, 
                         LOR_IRQFLAG_RxTimeout |
                         LOR_IRQFLAG_RxDone |
                         LOR_IRQFLAG_PayloadCrcError |
                         LOR_IRQFLAG_ValidHeader |
                         // LOR_IRQFLAG_TxDone |
                         LOR_IRQFLAG_CadDone |
                         // LOR_IRQFLAG_FhssChangeChannel |
                         LOR_IRQFLAG_CadDetected);

                // DIO0=TxDone, DIO2=FhssChangeChannel
                reg = sx1276_read_reg(dev, COM_RegDioMapping1);
                reg &= ~( (DOIMAPPING1_Dio0Mapping_MASK << 
                       DOIMAPPING1_Dio0Mapping_SHIFT) |
                       (DOIMAPPING1_Dio2Mapping_MASK << 
                       DOIMAPPING1_Dio2Mapping_SHIFT) );
                reg |= ( (DIOMAPPING_01 << DOIMAPPING1_Dio0Mapping_SHIFT) |
                       (DIOMAPPING_00 << DOIMAPPING1_Dio2Mapping_SHIFT) );
                sx1276_write_reg(dev, COM_RegDioMapping1, reg);
            } else {
                // mask out all except TxDone
                sx1276_write_reg(dev, LOR_RegIrqFlagsMask, 
                         LOR_IRQFLAG_RxTimeout |
                         LOR_IRQFLAG_RxDone |
                         LOR_IRQFLAG_PayloadCrcError |
                         LOR_IRQFLAG_ValidHeader |
                         // LOR_IRQFLAG_TxDone |
                         LOR_IRQFLAG_CadDone |
                         LOR_IRQFLAG_FhssChangeChannel |
                         LOR_IRQFLAG_CadDetected);

                // DIO0=TxDone
                reg = sx1276_read_reg(dev, COM_RegDioMapping1);
                reg &= ~( (DOIMAPPING1_Dio0Mapping_MASK << 
                       DOIMAPPING1_Dio0Mapping_SHIFT) );
                reg |= (DIOMAPPING_01 << DOIMAPPING1_Dio0Mapping_SHIFT);
                sx1276_write_reg(dev, COM_RegDioMapping1, reg);
            }
        }
        break;
    }

    dev->settings->state = STATE_TX_RUNNING;
    r_event = REVENT_EXEC;

    sx1276_set_op_mode(dev, MODE_TxMode);
    // find a workaround for this
    // get rid of init clock

    //uint32_t ns_s_time = SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32());
    //uint32_t ns_e_time = ns_s_time + 3000000000;
    int counter = 0;
    //while (SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32()) < ns_e_time && r_event == REVENT_EXEC);
    //while (ns_e_time && r_event == REVENT_EXEC)
printf("current state : %d\n", dev->settings->state);
    while (counter < 4000 && r_event != REVENT_DONE) {
        if(int_flag == 1) {
            sx1276_dio0_qos(dev);
            int_flag = int_flag & 0xfe;
        }
        if(int_flag == 2) {
            sx1276_dio1_qos(dev);
            int_flag = int_flag & 0xfd;
        }
        if(int_flag == 4) {
            sx1276_on_dio2_irq(dev);
            int_flag = int_flag & 0xfd;
        }
        if(int_flag == 8) {
            sx1276_on_dio3_irq(dev);
            int_flag = int_flag & 0xf7;
        }
        counter = counter + 1;
        //k_busy_wait(1000);
        upm_delay_ms(1);
        //k_sleep(1);
    }

uint8_t reg1 = sx1276_read_reg(dev, LOR_RegIrqFlags);
                    printf("IRQ flag values: %x\n", reg1);
       printf("r_event value after blah: %d\n", r_event);
    int_flag = 0;
    if (r_event == REVENT_EXEC) {
        // timeout
        printf("sx1276: setTx: Timeout occured\n");
        r_event = REVENT_TIMEOUT;
        dev->settings->state = STATE_IDLE;
        if((RadioEvents_c != NULL) && (RadioEvents_c->TxTimeout != NULL)) {
            RadioEvents_c->TxTimeout();
        }
    }

printf("exiting set tx function\n");
    //return UPM_SUCCESS;
    return r_event;
}

RADIO_EVENT_T sx1276_set_rx(sx1276_context dev, uint32_t timeout){
    bool rxContinuous = false;
    uint8_t reg = 0;

    switch (dev->settings->modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            // The datasheet does not mention anything other than an
            // InvertIQ bit (0x40) in RegInvertIQ register (0x33).  Here,
            // we seem to have two bits in RegInvertIQ (existing one for
            // RX), and a 'new' one for TXOff (0x01).  In addition,
            // INVERTIQ2 (0x3b) does not exist in the datasheet, it is
            // marked as reserved. We will assume that the datasheet is
            // out of date.
            if (dev->settings->loraSettings->IqInverted == true) {
                reg = sx1276_read_reg(dev, LOR_RegInvertIQ);
                reg &= ~(INVERTIQ_InvertIQTxOff | INVERTIQ_InvertIQRx);
                reg |= INVERTIQ_InvertIQRx;
                sx1276_write_reg(dev, LOR_RegInvertIQ, reg);

                // warning, hardcoded undocumented magic number into
                // undocumented register
                sx1276_write_reg(dev, LOR_RegInvertIQ2, 0x19);
            } else {
                reg = sx1276_read_reg(dev, LOR_RegInvertIQ);
                reg &= ~(INVERTIQ_InvertIQTxOff | INVERTIQ_InvertIQRx);
                reg |= INVERTIQ_InvertIQTxOff; // 'active' off.
                sx1276_write_reg(dev, LOR_RegInvertIQ, reg);

                // warning, hardcoded undocumented magic number into
                // undocumented register
                sx1276_write_reg(dev, LOR_RegInvertIQ2, 0x1d);
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if (dev->settings->loraSettings->Bandwidth < 9) {
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg &= 0x7f; // clear undocumented bit 7
                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);

                // warning, writing magic numbers into undocumented
                // registers
                switch (dev->settings->loraSettings->Bandwidth) {
                    case 0: // 7.8 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x48);
                        sx1276_set_channel(dev, dev->settings->channel + 7.81e3);
                    break;
                    case 1: // 10.4 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x44);
                        sx1276_set_channel(dev, dev->settings->channel + 10.42e3);
                    break;
                    case 2: // 15.6 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x44);
                        sx1276_set_channel(dev, dev->settings->channel + 15.62e3);
                    break;
                    case 3: // 20.8 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x44);
                        sx1276_set_channel(dev, dev->settings->channel + 20.83e3);
                    break;
                    case 4: // 31.2 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x44);
                        sx1276_set_channel(dev, dev->settings->channel + 31.25e3);
                    break;
                    case 5: // 41.4 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x44);
                        sx1276_set_channel(dev, dev->settings->channel + 41.67e3);
                    break;
                    case 6: // 62.5 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x40);
                    break;
                    case 7: // 125 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x40);
                    break;
                    case 8: // 250 kHz
                        sx1276_write_reg(dev, LOR_Reserved2f, 0x40);
                    break;
                }
            } else {
                reg = sx1276_read_reg(dev, LOR_RegDetectOptimize);
                reg |= 0x80; // set undocumented bit 7
                sx1276_write_reg(dev, LOR_RegDetectOptimize, reg);
            }

            rxContinuous = dev->settings->loraSettings->RxContinuous;

            if (dev->settings->loraSettings->FreqHopOn == true) {
                // mask out all except RxDone, RxTimeout, PayloadCrCError,
                // and FhssChangeChannel
                sx1276_write_reg(dev, LOR_RegIrqFlagsMask, 
                         // LOR_IRQFLAG_RxTimeout |
                         // LOR_IRQFLAG_RxDone |
                         // LOR_IRQFLAG_PayloadCrcError |
                         LOR_IRQFLAG_ValidHeader |
                         LOR_IRQFLAG_TxDone |
                         LOR_IRQFLAG_CadDone |
                         // LOR_IRQFLAG_FhssChangeChannel |
                         LOR_IRQFLAG_CadDetected);

                // DIO0=RxDone, DIO2=FhssChangeChannel
                reg = sx1276_read_reg(dev, COM_RegDioMapping1);
                reg &= ~( (DOIMAPPING1_Dio0Mapping_MASK << 
                       DOIMAPPING1_Dio0Mapping_SHIFT) |
                       (DOIMAPPING1_Dio2Mapping_MASK << 
                       DOIMAPPING1_Dio2Mapping_SHIFT) );
                reg |= ( (DIOMAPPING_00 << DOIMAPPING1_Dio0Mapping_SHIFT) |
                       (DIOMAPPING_00 << DOIMAPPING1_Dio2Mapping_SHIFT) );
                sx1276_write_reg(dev, COM_RegDioMapping1, reg);
            } else {
                // mask out all except RxDone, RxTimeout, and PayloadCrCError
                sx1276_write_reg(dev, LOR_RegIrqFlagsMask, 
                         // LOR_IRQFLAG_RxTimeout |
                         // LOR_IRQFLAG_RxDone |
                         // LOR_IRQFLAG_PayloadCrcError |
                         LOR_IRQFLAG_ValidHeader |
                         LOR_IRQFLAG_TxDone |
                         LOR_IRQFLAG_CadDone |
                         LOR_IRQFLAG_FhssChangeChannel |
                         LOR_IRQFLAG_CadDetected);

                // DIO0=RxDone
                reg = sx1276_read_reg(dev, COM_RegDioMapping1);
                reg &= ~(DOIMAPPING1_Dio0Mapping_MASK << 
                       DOIMAPPING1_Dio0Mapping_SHIFT);
                reg |= (DIOMAPPING_00 << DOIMAPPING1_Dio0Mapping_SHIFT);
                sx1276_write_reg(dev, COM_RegDioMapping1, reg);
            }

            sx1276_write_reg(dev, LOR_RegFifoRxBaseAddr, 0);
            sx1276_write_reg(dev, LOR_RegFifoAddrPtr, 0);
        }
        break;
        default:
        break;
    }

    memset(rx_buffer, 0, FIFO_SIZE);
    dev->settings->state = STATE_RX_RUNNING;
    r_event = REVENT_EXEC;

    if (dev->settings->modem == MODEM_FSK) {
        // currently not implemented 
    } else {
        if(rxContinuous == true) {
            sx1276_set_op_mode(dev, MODE_LOR_RxContinuous);
        } else {
            sx1276_set_op_mode(dev, MODE_LOR_RxSingle);
        }
    }

    int counter = 0;
    //while (SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32()) < ns_e_time && r_event == REVENT_EXEC)
    while (counter < 4000 && r_event == REVENT_EXEC) {
        counter = counter + 1;
        k_busy_wait(1000);
    }

    if (r_event == REVENT_EXEC) {
      // timeout
      r_event = REVENT_TIMEOUT;
      if((RadioEvents_c != NULL) && (RadioEvents_c->RxTimeout != NULL)) {
        RadioEvents_c->RxTimeout( );
      }
    }

    return r_event;
}

upm_result_t sx1276_read_fifo(sx1276_context dev, uint8_t* buffer, int len){
    // can't read more than 256 bytes
    if (len > FIFO_SIZE)
    {
        printf("sx1276: cannot read more than 256 bytes from FIFO\n");
        return UPM_ERROR_OPERATION_FAILED;
    }

    uint8_t pkt = 0;

    sx1276_cs_on(dev);
    if (mraa_spi_transfer_buf(dev->spi, &pkt, NULL, 1) != MRAA_SUCCESS) {
        printf("sx1276: Spi.transfer(0) failed\n");
        sx1276_cs_off(dev);
        return UPM_ERROR_OPERATION_FAILED;
    }

    if (mraa_spi_transfer_buf(dev->spi, NULL, buffer, len)) {
        printf("sx1276: Spi.transfer(buf) failed\n");
        sx1276_cs_off(dev);
        return UPM_ERROR_OPERATION_FAILED;
    }
    sx1276_cs_off(dev);

    return UPM_SUCCESS;
}

upm_result_t sx1276_dio0_qos(sx1276_context dev) {
    volatile uint8_t irqFlags = 0;
    switch (dev->settings->state) {
        case STATE_RX_RUNNING: {
            switch (dev->settings->modem) {
                case MODEM_FSK: {
                }
                break;
                case MODEM_LORA: {
                    int8_t snr = 0;
                    sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_RxDone);
                    irqFlags = sx1276_read_reg(dev, LOR_RegIrqFlags);

                    // cerr << "LORA PayloadCRC on = " 
                    //      <<  hex << (int)This->readReg(LOR_RegHopChannel) << dec << endl;
                    if (irqFlags & LOR_IRQFLAG_PayloadCrcError) {
                        // Clear Irq
                        sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_PayloadCrcError);
                        if (dev->settings->loraSettings->RxContinuous == false) {
                            dev->settings->state = STATE_IDLE;
                        }
                        // RxError radio event
                        // cerr << __FUNCTION__ << ": RxError (payload crc error)" << endl;
                        // adding callback function if one is linked
                        if((RadioEvents_c != NULL) && (RadioEvents_c->RxError != NULL)) {
                            RadioEvents_c->RxError();
                        }
                        r_event = REVENT_ERROR;

                        break;
                    }

                    dev->settings->loraPacketHandler->SnrValue = sx1276_read_reg(dev, LOR_RegPktSnrValue);
                    if (dev->settings->loraPacketHandler->SnrValue & 0x80) {
                        // The SNR sign bit is 1
                        // Invert and divide by 4
                        snr = ( (~(dev->settings->loraPacketHandler->SnrValue) + 1 ) & 0xff) >> 2;
                        snr = -snr;
                    } else {
                        // Divide by 4
                        snr = (dev->settings->loraPacketHandler->SnrValue & 0xff) >> 2;
                    }

                    int16_t rssi = sx1276_read_reg(dev, LOR_RegPktRssiValue);

                    if (snr < 0) {
                        if (dev->settings->channel > RF_MID_BAND_THRESH) {
                            dev->settings->loraPacketHandler->RssiValue =
                            LOR_RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) + snr;
                        } else {
                            dev->settings->loraPacketHandler->RssiValue =
                            LOR_RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + snr;
                        }
                    } else {
                        if (dev->settings->channel > RF_MID_BAND_THRESH) {
                            dev->settings->loraPacketHandler->RssiValue =
                            LOR_RSSI_OFFSET_HF + rssi + (rssi >> 4);
                        } else {
                            dev->settings->loraPacketHandler->RssiValue =
                            LOR_RSSI_OFFSET_LF + rssi + (rssi >> 4);
                        }
                    }

                    dev->settings->loraPacketHandler->Size = 
                    sx1276_read_reg(dev, LOR_RegRxNbBytes);

                    // cerr << "LORA HANDLER SIZE = " 
                    //      <<  (int)This->m_settings.loraPacketHandler.Size << endl;

                    // cerr << "LORA MAXPAYLOAD = " 
                    //      <<  (int)This->readReg(LOR_RegMaxPayloadLength) << endl;

                    sx1276_read_fifo(dev, rx_buffer, dev->settings->loraPacketHandler->Size);

                    if (dev->settings->loraSettings->RxContinuous == false) {
                        dev->settings->state = STATE_IDLE;
                    }

                    // RxDone radio event

                    // The returned size (from LOR_RegRxNbBytes) is always 64
                    // bytes regardless of the packet size I sent.  Something
                    // is wrong here.
                    //            cerr << __FUNCTION__ << ": RxDone (LORA)" << endl;
                    rx_RSSI = (int)rssi;
                    rx_SNR = (int)snr;
                    rx_Len = dev->settings->loraPacketHandler->Size;

                    // adding callback function here if there is one linked
                    if (RadioEvents_c->RxDone != NULL)
                       RadioEvents_c->RxDone(rx_buffer, dev->settings->loraPacketHandler->Size, dev->settings->loraPacketHandler->RssiValue, dev->settings->loraPacketHandler->SnrValue);

                    r_event = REVENT_DONE;
                    // if (This->m_settings.state == STATE_RX_RUNNING)
                    //   fprintf(stderr, "### %s: snr = %d rssi = %d RX(%d): %s\n", 
                    //           __FUNCTION__, 
                    //           (int)snr, (int)rssi,
                    //           This->m_settings.loraPacketHandler.Size,
                    //           This->m_rxBuffer);
                    // else
                    //   fprintf(stderr, "### %s: snr = %d rssi = %d RX: INV BUFFER (crc)\n", __FUNCTION__, 
                    //         (int)snr, (int)rssi);
                }
                break;
//            }
                default:
                break;
            }
        }
        break;

        case STATE_TX_RUNNING: {
            // TxDone interrupt
            switch (dev->settings->modem) {
                case MODEM_LORA: {
                    //printf("sx1276: dio0:asrtgfdsawdethryjhnfgbds\n");
                    // Clear Irq
                    //uint8_t reg = sx1276_read_reg(l_dev, LOR_RegIrqFlags);
                    //printf("IRQ flag values: %x\n", reg);
                    
                    sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_TxDone);
                    // fprintf(stderr, "%s: LORA IrqFlags = %02x\n", __FUNCTION__,
                    //         This->readReg(LOR_RegIrqFlags));
                    // Intentional fall through
                }
                case MODEM_FSK:
                default:
                    dev->settings->state = STATE_IDLE;

                    // adding callback function here if there is one linked
                    if((RadioEvents_c != NULL) && (RadioEvents_c->TxDone != NULL)) {
                        RadioEvents_c->TxDone();
                    }

                    // TxDone radio event
                    r_event = REVENT_DONE;
                    //          cerr << __FUNCTION__ << ": TxDone" << endl;
                break;
            }
            default:
            break;
        }
    }
    // unlock other interrupts
    //printf("current state inside interrupt: %d\n", l_dev->settings->state);
    //switch (This->m_settings.state)
    return UPM_SUCCESS;
}

upm_result_t sx1276_dio1_qos(sx1276_context dev) {
    // lock all the interrupts
    switch (dev->settings->state) {
        case STATE_RX_RUNNING: {
            switch (dev->settings->modem) {
                case MODEM_FSK: {
                }
                break;
                case MODEM_LORA: {
                    // Sync time out
                    dev->settings->state = STATE_IDLE;
                    // RxError (LORA timeout) radio events
                    //          cerr << __FUNCTION__ << ": RxTimeout (LORA)" << endl;
                    r_event = REVENT_TIMEOUT;
                }
                break;
                default:
                break;
            }
        }
        break;
        case STATE_TX_RUNNING: {
            switch(dev->settings->modem) {
                case MODEM_FSK: {
                }
                break;
                case MODEM_LORA: {
                }
                break;
                default:
                break;
            }
        }
        break;
        default:
        break;
    }
    printf("r_event: %d\n", r_event);
    return UPM_SUCCESS;
}

upm_result_t sx1276_dio2_qos(sx1276_context dev) {
    switch(dev->settings->state) {
        case STATE_RX_RUNNING: {
            switch(dev->settings->modem) {
                case MODEM_FSK: {
                }
                break;
                case MODEM_LORA: {
                    if (dev->settings->loraSettings->FreqHopOn == true) {
                        // Clear Irq
                        sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_FhssChangeChannel);

                        // Fhss radio event (unsupported currently)
                        // FhssChangeChannel( (readReg( LOR_RegHopChannel) & 
                        // ~_HOPCHANNEL_FhssPresentChannel_MASK) );
                        //cerr << __FUNCTION__ << ": Fhss Change Channel (LORA, RX running)" << endl;

                        // add callback if one is linked
                        if((RadioEvents_c != NULL) && (RadioEvents_c->FhssChangeChannel != NULL)) {
                            RadioEvents_c->FhssChangeChannel((sx1276_read_reg(dev, LOR_RegHopChannel) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                }
                break;
                default:
                break;
            }
        }
        break;
        case STATE_TX_RUNNING: {
            switch(dev->settings->modem) {
                case MODEM_FSK: {
                }
                break;
                case MODEM_LORA: {
                    if (dev->settings->loraSettings->FreqHopOn == true) {
                        // Clear Irq
                        sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_FhssChangeChannel);

                        // Fhss radio event (unsupported currently)
                        // FhssChangeChannel( (readReg( LOR_RegHopChannel) & 
                        // ~_HOPCHANNEL_FhssPresentChannel_MASK) );
                        //cerr << __FUNCTION__ << ": Fhss Change Channel (LORA, TX running)" << endl;
                        // add callback if one is linked
                        if((RadioEvents_c != NULL) && (RadioEvents_c->FhssChangeChannel != NULL)) {
                            RadioEvents_c->FhssChangeChannel((sx1276_read_reg(dev, LOR_RegHopChannel) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                }
                break;
                default:
                break;
            }
        }
        break;
        default:
        break;
    }
    // unlock interrupts
    return UPM_SUCCESS;
}

upm_result_t sx1276_dio3_qos(sx1276_context dev) {
    switch (dev->settings->modem) {
        case MODEM_FSK: {
        }
        break;
        case MODEM_LORA: {
            if (sx1276_read_reg(dev, LOR_RegIrqFlags) & LOR_IRQFLAG_CadDetected) {
                // Clear Irq
                sx1276_write_reg(dev, LOR_RegIrqFlags, 
                         (LOR_IRQFLAG_CadDetected | LOR_IRQFLAG_CadDone));

                // CADDetected radio event (true)
                // add callback only if linked
                if((RadioEvents_c != NULL) && (RadioEvents_c->CadDone != NULL)) {
                    RadioEvents_c->CadDone(true);
                }
                // cerr << __FUNCTION__ << ": CadDetected (LORA)" << endl;

            } else {
                // Clear Irq
                sx1276_write_reg(dev, LOR_RegIrqFlags, LOR_IRQFLAG_CadDone);
                // CADDetected radio event (false)
                if((RadioEvents_c != NULL) && (RadioEvents_c->CadDone != NULL)) {
                    RadioEvents_c->CadDone(false);
                }
                //cerr << __FUNCTION__ << ": CadDone (LORA)" << endl;
            }
        }
        break;
        default:
        break;	
    }
    return UPM_SUCCESS;
}

void sx1276_on_dio0_irq() {
    printf("interrupt 0 detected\n");
    int_flag = int_flag | 1;
}

void sx1276_on_dio1_irq() {
    printf("interrupt 1 detected\n");
    int_flag = int_flag | 2;
}

void sx1276_on_dio2_irq() {
    printf("interrupt 2 detected\n");
    int_flag = int_flag | 4;
}

void sx1276_on_dio3_irq() {
    printf("interrupt 3 detected\n");
    int_flag = int_flag | 8;
}

void sx1276_on_dio4_irq() {
    printf("interrupt 4 detected\n");
    // not doing anything
    // unlock interrupts
}

upm_result_t sx1276_cs_on(sx1276_context dev) {
    if(mraa_gpio_write(dev->gpio_cs, 0) != MRAA_SUCCESS) {
        printf("unable to select chip\n");
    }
    return UPM_SUCCESS;
}
upm_result_t sx1276_cs_off(sx1276_context dev) {
    if(mraa_gpio_write(dev->gpio_cs, 1) != MRAA_SUCCESS) {
        printf("unable to select chip\n");
    }
    return UPM_SUCCESS;
}

upm_result_t sx1276_set_sleep(sx1276_context dev) {
    sx1276_set_op_mode(dev, MODE_Sleep);
    dev->settings->state = STATE_IDLE;
    return UPM_SUCCESS;
}

void sx1276_set_max_payload_length(sx1276_context dev, RADIO_MODEM_T modem, uint8_t max){
    sx1276_set_modem(dev, modem);

    switch(modem) {
        case MODEM_FSK:
        break;
        case MODEM_LORA:
            sx1276_write_reg(dev, LOR_RegMaxPayloadLength, max);
        break;
    }
}

void sx1276_set_public_network(sx1276_context dev, bool enable) {
    sx1276_set_modem(dev, MODEM_LORA);
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        sx1276_write_reg(dev, LOR_RegSyncWord, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        // Change LoRa modem SyncWord
        sx1276_write_reg(dev, LOR_RegSyncWord, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

RADIO_STATES_T sx1276_get_status(sx1276_context dev) {
    return dev->settings->state;
}

bool sx1276_check_rf_frequency(sx1276_context dev, uint32_t freq) {
    // currently all frequencies are supported as per semtech sx1276 driver
    return true;
}

uint32_t sx1276_random(sx1276_context dev) {
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    sx1276_set_modem(dev, MODEM_LORA);

    // Disable LoRa modem interrupts
    sx1276_write_reg(dev, LOR_RegIrqFlagsMask, LOR_IRQFLAG_RxTimeout |
                LOR_IRQFLAG_RxDone |
                LOR_IRQFLAG_PayloadCrcError |
                LOR_IRQFLAG_ValidHeader |
                LOR_IRQFLAG_TxDone |
                LOR_IRQFLAG_CadDone |
                LOR_IRQFLAG_FhssChangeChannel |
                LOR_IRQFLAG_CadDetected );

    // Set radio in continuous reception
    sx1276_set_op_mode(dev, MODE_LOR_RxContinuous);

    for(i=0;i<32;i++) {
        //DelayMs( 1 );
        upm_delay_ms(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ((uint32_t)sx1276_read_reg(dev, LOR_RegRssiWideband)&0x01) << i;
    }

    sx1276_set_sleep(dev);

    return rnd;
}

uint32_t sx1276_get_time_on_air(sx1276_context dev, RADIO_MODEM_T modem, uint8_t pktLen) {
    uint32_t airTime = 0;

    switch(modem) {
        case MODEM_FSK:{}
        break;
        case MODEM_LORA: {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch(dev->settings->loraSettings->Bandwidth) {
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
            double rs = bw / (1 << dev->settings->loraSettings->Datarate);
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( dev->settings->loraSettings->PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil((8 * pktLen - 4 * dev->settings->loraSettings->Datarate +
                              28 + 16 * dev->settings->loraSettings->CrcOn -
                              (dev->settings->loraSettings->FixLen ? 20 : 0))/
                              (double)(4 * (dev->settings->loraSettings->Datarate -
                              ((dev->settings->loraSettings->LowDatarateOptimize > 0) ? 2:0))))*
                              (dev->settings->loraSettings->Coderate + 4);
            double nPayload = 8 + ((tmp > 0) ? tmp:0);
            double tPayload = nPayload*ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor(tOnAir*1e3 + 0.999);
        }
        break;
    }

    return airTime;
}

void sx1276_set_tx_continuous_wave(sx1276_context dev, uint32_t freq, int8_t power, uint16_t time) {
    uint32_t timeout = ( uint32_t )( time * 1e3 );

    sx1276_set_channel(dev, freq);
    //SX1276SetChannel( freq );

    // check what has to be done with the timeout
    sx1276_set_tx_config(dev, MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0);
    //SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    sx1276_write_reg(dev, FSK_RegPacketConfig2, (sx1276_read_reg(dev, FSK_RegPacketConfig2) & COM_PacketConfig2_DataMode_Mask));
    //SX1276Write( FSK_RegPacketConfig2, ( SX1276Read( FSK_RegPacketConfig2 ) & COM_PacketConfig2_DataMode_Mask ) );
    // Disable radio interrupts
    sx1276_write_reg(dev, COM_RegDioMapping1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    sx1276_write_reg(dev, COM_RegDioMapping2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);
    //SX1276Write( COM_RegDioMapping1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    //SX1276Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    //TimerSetValue( &TxTimeoutTimer, timeout );

    //SX1276.Settings.State = RF_TX_RUNNING;
    //TimerStart( &TxTimeoutTimer );
    int counter = 0;
    dev->settings->state = STATE_TX_RUNNING;
    r_event = REVENT_EXEC;
    //k_timer_start(&TxTimeoutTimer, timeout, 0);
    //SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
    sx1276_set_op_mode(dev, MODE_TxMode);

    while (counter < timeout && r_event != REVENT_DONE) {
        if(int_flag == 1) {
            sx1276_dio0_qos(dev);
            int_flag = int_flag & 0xfe;
        }
        if(int_flag == 2) {
            sx1276_dio1_qos(dev);
            int_flag = int_flag & 0xfd;
        }
        if(int_flag == 4) {
            sx1276_on_dio2_irq(dev);
            int_flag = int_flag & 0xfd;
        }
        if(int_flag == 8) {
            sx1276_on_dio3_irq(dev);
            int_flag = int_flag & 0xf7;
        }
        counter = counter + 1;
        //k_busy_wait(1000);
        upm_delay_ms(1);
        //k_sleep(1);
    }

    // if the state hasnt changed so far we are in for a timeout
    if(r_event == REVENT_EXEC) {
        dev->settings->state = STATE_IDLE;
        if((RadioEvents_c != NULL) && (RadioEvents_c->TxTimeout != NULL)) {
            RadioEvents_c->TxTimeout();
        }
    }
}

void sx1276_set_standby(sx1276_context dev) {
    sx1276_set_op_mode(dev, MODE_Standby );
    dev->settings->state = STATE_IDLE;
}

/*
void sx1276_on_timeout_irq(sx1276_context dev)
{
    switch( SX1276.Settings.State )
    {
    case RF_RX_RUNNING:
 NOT INTERESTED FOR NOW
        if( SX1276.Settings.Modem == MODEM_FSK )
        {
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1276Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( SX1276.Settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                TimerStart( &RxTimeoutSyncWord );
            }
            else
            {
                SX1276.Settings.State = RF_IDLE;
                TimerStop( &RxTimeoutSyncWord );
            }
        }
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        SX1276.Settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}
*/
