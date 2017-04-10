#pragma once

// fsk - 0x12 -x13
#define REG_RXBW                 0x12
#define REG_AFCBW                0x13
// Packet handler settings
#define REG_PREAMBLEMSB          0x25
#define REG_PREAMBLELSB          0x26
#define REG_FSK_PAYLOADLENGTH    0x32
#define REG_PACKETCONFIG1        0x30
#define REG_PACKETCONFIG2        0x31
#define REG_PREAMBLE_DETECT      0x1F
#define REG_OSC                  0x24
#define REG_FDEVMSB              0x04
#define REG_FDEVLSB              0x05

/*!
 * RegPacketConfig1
 */
#define RF_PACKETCONFIG1_PACKETFORMAT_MASK          0x7F
#define RF_PACKETCONFIG1_PACKETFORMAT_FIXED         0x00
#define RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE      0x80  // Default

#define RF_PACKETCONFIG1_DCFREE_MASK                0x9F
#define RF_PACKETCONFIG1_DCFREE_OFF                 0x00  // Default
#define RF_PACKETCONFIG1_DCFREE_MANCHESTER          0x20
#define RF_PACKETCONFIG1_DCFREE_WHITENING           0x40

#define RF_PACKETCONFIG1_CRC_MASK                   0xEF
#define RF_PACKETCONFIG1_CRC_ON                     0x10  // Default
#define RF_PACKETCONFIG1_CRC_OFF                    0x00

#define RF_PACKETCONFIG1_CRCAUTOCLEAR_MASK          0xF7
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_ON            0x00  // Default
#define RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF           0x08

#define RF_PACKETCONFIG1_ADDRSFILTERING_MASK        0xF9
#define RF_PACKETCONFIG1_ADDRSFILTERING_OFF         0x00  // Default
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODE        0x02
#define RF_PACKETCONFIG1_ADDRSFILTERING_NODEBROADCAST 0x04

#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_MASK      0xFE
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT     0x00  // Default
#define RF_PACKETCONFIG1_CRCWHITENINGTYPE_IBM       0x01

/*!
 * RegPacketConfig2
 */

#define RF_PACKETCONFIG2_WMBUS_CRC_ENABLE_MASK      0x7F
#define RF_PACKETCONFIG2_WMBUS_CRC_ENABLE           0x80
#define RF_PACKETCONFIG2_WMBUS_CRC_DISABLE          0x00  // Default

#define RF_PACKETCONFIG2_DATAMODE_MASK              0xBF
#define RF_PACKETCONFIG2_DATAMODE_CONTINUOUS        0x00
#define RF_PACKETCONFIG2_DATAMODE_PACKET            0x40  // Default

#define RF_PACKETCONFIG2_IOHOME_MASK                0xDF
#define RF_PACKETCONFIG2_IOHOME_ON                  0x20
#define RF_PACKETCONFIG2_IOHOME_OFF                 0x00  // Default

#define RF_PACKETCONFIG2_BEACON_MASK                0xF7
#define RF_PACKETCONFIG2_BEACON_ON                  0x08
#define RF_PACKETCONFIG2_BEACON_OFF                 0x00  // Default

#define RF_PACKETCONFIG2_PAYLOADLENGTH_MSB_MASK 0xF8
