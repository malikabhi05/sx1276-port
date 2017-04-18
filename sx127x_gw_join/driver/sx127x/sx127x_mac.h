#pragma once

#include "upm.h"
#include "mraa.h"
#include "sx127x.h"

#define RECEIVE_DELAY1            1        // s
#define RECEIVE_DELAY2            2        // s (must be RECEIVE_DELAY1 + 1s)
#define JOIN_ACCEPT_DELAY1        5        // s
#define JOIN_ACCEPT_DELAY2        6        // s
#define MAX_FCNT_GAP              16384
#define ADR_ACK_LIMIT             64
#define ADR_ACK_DELAY             32
#define ACK_TIMEOUT               2        // +/-1 s

// this is specific to the US 915 standard
#define LORA_MAX_NB_CHANNELS      72
#define LORA_MIN_NB_CHANNELS      0

// vv imp
/*!
 * LoRaMac datarates definition
 */
#define DR_0                                        0  // SF10 - BW125 |
#define DR_1                                        1  // SF9  - BW125 |
#define DR_2                                        2  // SF8  - BW125 +-> Up link
#define DR_3                                        3  // SF7  - BW125 |
#define DR_4                                        4  // SF8  - BW500 |
#define DR_5                                        5  // RFU
#define DR_6                                        6  // RFU
#define DR_7                                        7  // RFU
#define DR_8                                        8  // SF12 - BW500 |
#define DR_9                                        9  // SF11 - BW500 |
#define DR_10                                       10 // SF10 - BW500 |
#define DR_11                                       11 // SF9  - BW500 |
#define DR_12                                       12 // SF8  - BW500 +-> Down link
#define DR_13                                       13 // SF7  - BW500 |
#define DR_14                                       14 // RFU          |
#define DR_15                                       15 // RFU          |

static int8_t dr_table[16][2] = 
{
    {10, 0},           // DR_0
    {9, 0},            // DR_1
    {8, 0},            // DR_2
    {7, 0},            // DR_3
    {8, 2},            // DR_4
    {-1, -1},          // DR_5
    {-1, -1},          // DR_6
    {-1, -1},          // DR_7
    {12, 2},           // DR_8
    {11, 2},           // DR_9
    {10, 2},           // DR_10
    {9, 2},            // DR_11
    {8, 2},            // DR_12
    {7, 2},            // DR_13
    {-1, -1},          // DR_14
    {-1, -1}           // DR_15
};

/*!
 * Up/Down link data rates offset definition
 */
static int8_t data_rate_offsets[5][4] =
{
    { DR_10, DR_9 , DR_8 , DR_8  }, // DR_0
    { DR_11, DR_10, DR_9 , DR_8  }, // DR_1
    { DR_12, DR_11, DR_10, DR_9  }, // DR_2
    { DR_13, DR_12, DR_11, DR_10 }, // DR_3
    { DR_13, DR_13, DR_12, DR_11 }, // DR_4
};

// this can be between DR_0 through DR_4
static int8_t uplink_datarate;
// this is a function of the uplink datarate
static int8_t downlink_datarate_1;
// as per spec this datarate is fixed
static int8_t downlink_datarate_2 = DR_8;

/*!
 * LoRaMAC channels parameters definition
 */
typedef struct uDrRange
{
    /*!
     * Byte-access to the bits
     */
    //int8_t Value;
    /*!
     * Structure to store the minimum and the maximum datarate
     */
    //struct sFields
    //{
         /*!
         * Minimum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Min;
        /*!
         * Maximum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Max;
    //}Fields;
}DrRange_t;

/*!
 * LoRaMAC channel definition
 */
typedef struct sChannelParams
{
    /*!
     * Frequency in Hz
     */
    uint32_t Frequency;
    /*!
     * Data rate definition
     */
    DrRange_t DrRange;
    /*!
     * Band index
     */
    uint8_t Band;
}ChannelParams_t;

typedef struct uMHDR {
    uint8_t type;
    uint8_t RFU;
    uint8_t Major;
} MHDR_t;

static MHDR_t MHDR;

/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS];

upm_result_t lorawan_init();

upm_result_t lorawan_otaa_join();

uint8_t lorawan_set_random_channel();

void lorawan_create_package();

void lorawan_set_dev_eui(uint8_t* dev_eui);

void lorawan_set_app_eui(uint8_t* app_eui);

void lorawan_set_app_key(uint8_t* app_key);
