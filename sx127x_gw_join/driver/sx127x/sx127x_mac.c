#include "sx127x_mac.h"
#include "zephyr.h"
#include <sys_clock.h>
#include "kernel.h"
#include "device.h"

#include "sx127x.h"
#include "unistd.h"
#include "mraa.h"
#include "loramac-definitions.h"
#include "loramaccrypto.h"
#include "loramactest.h"

uint8_t pack[23];

uint8_t rx_buffer[256] = {0};

uint8_t DEVEUI[8] = {0x00};

uint8_t APPEUI[8] = {0x00};

uint8_t APPKEY[16] = {0x00};

upm_result_t lorawan_init() {
   /*!
    * LoRaMac default channels
    */
   // Channel = { Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
   /*
    * US band channels are initialized using a loop in LoRaMacInit function
    * \code
    */
    // 125 kHz channels
    for( uint8_t i = 0; i < LORA_MAX_NB_CHANNELS - 8; i++ ) {
        Channels[i].Frequency = 902.3e6 + i * 200e3;
        Channels[i].DrRange.Min = DR_0;
        Channels[i].DrRange.Max = DR_3;
        Channels[i].Band = 0;
    }
    // 500 kHz channels
    for( uint8_t i = LORA_MAX_NB_CHANNELS - 8; i < LORA_MAX_NB_CHANNELS; i++ ) {
        Channels[i].Frequency = 903.0e6 + ( i - ( LORA_MAX_NB_CHANNELS - 8 ) ) * 1.6e6;
        Channels[i].DrRange.Min = DR_4;
        Channels[i].DrRange.Max = DR_4;
        Channels[i].Band = 0;
    }

    LoRaClass();
    setPins(60, 82, 72);

    init(9041e5);
}

uint8_t lorawan_set_random_channel() {
    return (rand() % ((LORA_MAX_NB_CHANNELS-1) + 1 - LORA_MIN_NB_CHANNELS)) + LORA_MIN_NB_CHANNELS;
}

upm_result_t lorawan_otaa_join() {
    uint32_t mic = 0;
    // since this is join
    pack[0] = 0;
    memcpy(pack+1, APPEUI, 8);
    memcpy(pack+9, DEVEUI, 8);

    uint16_t nonce = SX1276Random();
    pack[17] = nonce & 0xff;
    pack[18] = (nonce >> 8) & 0xff;

    LoRaMacJoinComputeMic(pack, 19, APPKEY, &mic);

    pack[19] = mic&0xff;
    pack[20] = (mic >> 8) & 0xff;
    pack[21] = (mic >> 16) & 0xff;
    pack[22] = (mic >> 24) & 0xff;

    int c = 0;
    for(c=0; c<23; c++)
        printf("%x, ", pack[c]);
    printf("\n");

    // hard-coded for now, will have to change
    setFrequency(9043e5);

    SX1276SetTxConfig(MODEM_LORA, 20, 0, dr_table[0][1], dr_table[0][0], 1, 8, false, true, 0, 0, false, 3000);
    SX1276SetPublicNetwork(true);
    SX1276Send(pack, 23);

#if 1
    printf("*********************************************************\n");
    printf("************************ Enter RX ***********************\n");
    printf("*********************************************************\n");
    upm_delay_us(4000000);
    // opening up the first rx window, if the downlink packet is received in this window then
    // we wont go to the second rx window. The wait should be 5 seconds between tx and rx1 window 1
    // but I'm paranoid and therefore will open it up 4 seconds after and will keep it open for 8 seconds
    // pointlessly.
    // again hard-coded, will have to change, this and the above tx freq form a pair of sorts
    setFrequency(9245e5);

    SX1276SetRxConfig(MODEM_LORA, 2, 10,
                      1, 0, 8,
                      8, false,
                      0, false, 0, 0, true, true);

    SX1276SetRx(8000);

    int rx_buf_len = SX1276GetRxBufferlen();
    SX1276GetRxBuffer(rx_buffer);
    int x = 0;
    for (x = 0; x < rx_buf_len; x++)
        printf("%x, ", rx_buffer[x]);
    printf("\n");
#endif
}

void lorawan_set_dev_eui(uint8_t* dev_eui) {
    int c = 0;
    for(c=0; c<8; c++)
        DEVEUI[c] = dev_eui[c];
}

void lorawan_set_app_eui(uint8_t* app_eui) {
    int c = 0;
    for(c=0; c<8; c++)
        APPEUI[c] = app_eui[c];
}

void lorawan_set_app_key(uint8_t* app_key) {
    int c = 0;
    for(c=0; c<16; c++)
        APPKEY[c] = app_key[c];
}
