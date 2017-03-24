
#include <zephyr.h>
#include <misc/printk.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>
#include <math.h>
#include "Commissioning.h"
#include "loramac.h"
#include "upm.h"
#include "mraa.h"
#include "mraa/gpio.h"
#include "mraa/spi.h"
#include "sx1276.h"
#include "upm_utilities.h"

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_433 ) || defined( USE_BAND_470 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
struct k_timer TxNextPacketTimer;
//static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

struct k_timer l_timer;

void timer_delay_local(int time_ms) {
    k_timer_init(&l_timer, NULL, NULL);
    k_timer_start(&l_timer, time_ms, 0);
    while(k_timer_status_get(&l_timer) == 0);
}

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 2:
        {
#if defined( USE_BAND_433 ) || defined( USE_BAND_470 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
            //uint16_t pressure = 0;
            //int16_t altitudeBar = 0;
            //int16_t temperature = 0;
            //int32_t latitude, longitude = 0;
            //int16_t altitudeGps = 0xFFFF;
            //uint8_t batteryLevel = 0;

            //pressure = ( uint16_t )( MPL3115ReadPressure( ) / 10 );             // in hPa / 10
            //temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100
            //altitudeBar = ( int16_t )( MPL3115ReadAltitude( ) * 10 );           // in m * 10
            //batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            //GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            //altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = 0;
            AppData[1] = 1;
            AppData[2] = 2;
            AppData[3] = 3;
            AppData[4] = 4;
            AppData[5] = 5;
            AppData[6] = 6;
            AppData[7] = 7;
            AppData[8] = 8;
            AppData[9] = 9;
            AppData[10] = 10;
            AppData[11] = 11;
            AppData[12] = 12;
            AppData[13] = 13;
            AppData[14] = 14;
            AppData[15] = 15;
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
            //int16_t temperature = 0;
            //int32_t latitude, longitude = 0;
            //uint16_t altitudeGps = 0xFFFF;
            //uint8_t batteryLevel = 0;

            //temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100

            //batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            //GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            //altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = 10;
            AppData[1] = 9;                                           // Signed degrees celsius in half degree units. So,  +/-63 C
            AppData[2] = 8;                                          // Per LoRaWAN spec; 0=Charging; 1...254 = level, 255 = N/A
            AppData[3] = 7;
            AppData[4] = 6;
            AppData[5] = 5;
            AppData[6] = 4;
            AppData[7] = 3;
            AppData[8] = 2;
            AppData[9] = 1;
            AppData[10] = 0;
#endif
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
printf("part 1\n");
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
printf("part 2\n");
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
printf("part 3\n");
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }
printf("before send frame\n");
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
printf("successful return\n");
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent(struct k_timer *timer)
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;
//printf("tx timer expired\n");
    //TimerStop( &TxNextPacketTimer );
    // adding zephyr k_timer
    k_timer_stop(&TxNextPacketTimer);

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                    //GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    //GpsStart( );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        //GpsStart( );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
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
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{

upm_delay_ms(1000);
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;
MibRequestConfirm_t mibReq1;
    LoRaMacStatus_t status;

    //BoardInitMcu( );
    //BoardInitPeriph( );

    DeviceState = DEVICE_STATE_INIT;

//case DEVICE_STATE_INIT:
 //           {
printf("entering device state init\n");
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                //LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                //TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
                //k_timer_init(&TxNextPacketTimer, OnTxNextPacketTimerEvent, NULL);

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
                LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

                mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
                mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_RX2_CHANNEL;
                mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
                DeviceState = DEVICE_STATE_JOIN;
                //break;
 //           }


//case DEVICE_STATE_JOIN:
  //          {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }
                //DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    // these numbers are st micro specific, however they there is no real
                    // impact as such, they can completely be removed too
                    srand(0x1FF800D0^0x1FF800D4^0x1FF800E4);

                    // Choose a random device address
                    DevAddr = upm_rand_range( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                //DeviceState = DEVICE_STATE_SEND;
#endif
        //        break;
        
#if 0

    while( 1 )
    {
        //switch( DeviceState )
        //{
            
            
          //  case DEVICE_STATE_SEND:
           // {
                if( NextTx == true )
                {
//printf("onto the next transaction\n");
                    PrepareTxFrame( AppPort );
//printf("next frame prepared\n");
                    NextTx = SendFrame( );
//printf("next frame sent\n");
                }
/*
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + upm_rand_range( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
//printf("time before next send: %d\n", TxDutyCycleTime);
                }
*/

                //DeviceState = DEVICE_STATE_CYCLE;
//printf("onto the device state cycle\n");
                //break;
            //}
            //case DEVICE_STATE_CYCLE:
            //{
//printf("coming into device state cycle\n");
                //DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                //TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                //TimerStart( &TxNextPacketTimer );
                //k_timer_start(&TxNextPacketTimer, TxDutyCycleTime, 0);
////////////////////////////////////////////////////////////////////////////////////
                    
//printf("tx timer expired\n");
    //TimerStop( &TxNextPacketTimer );
    // adding zephyr k_timer
    //k_timer_stop(&TxNextPacketTimer);
//k_busy_wait(5000);
timer_delay_local(500);
    mibReq1.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq1 );
    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq1.Param.IsNetworkJoined == true )
        {
printf("sending\n");
            //DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
printf("trying to join again\n");
            //DeviceState = DEVICE_STATE_JOIN;
        }
    }
///////////////////////////////////////////////////////////////////////////////////////
              //  break;
            //}
            
            
        //}
//upm_delay_ms(10);
printf("remaining time: %d\n", DeviceState);
    }
#endif

while(1) {
    timer_delay_local(1000);
    PrepareTxFrame( AppPort );
    NextTx = SendFrame();
    //timer_delay_local(500);
    mibReq1.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq1 );
    printf("hellow!! status: %d\n", status);
}
}

/*
uint8_t arr[4] = {0};

char __noinit __stack thread_stack_area[1024];
struct k_timer timer;

void
mraa_sleep_ms(int ms)
{
    k_timer_init(&timer, NULL, NULL);
    k_timer_start(&timer, K_MSEC(500), 0);
    k_timer_status_sync(&timer);
}

/////////////////////////////////////////////////////////////////

//void thread_interrupt(void *dummy1, void *dummy2, void *dummy3);
void thread_functions(void *dummy1, void *dummy2, void *dummy3);
/////////////////////////////////////////////////////////////////

void main(void)
{
upm_delay_ms(500);


    // init


    sx1276_context dev = sx1276_init(1, 60, 82, 72, 74, 70, 76, 78, 80);
    if(dev == NULL) {
        printf("issues in sx1276 driver\n");
    }

int_flag = 0;
//mraa_sleep_ms(500);
    //sx1276_context dev = (sx1276_context) dummy1;
    if(sx1276_set_channel(dev, 915000000) != UPM_SUCCESS) {
        printf("issues setting the channel\n");
    }

    sx1276_set_tx_config(dev, MODEM_LORA, 14, 0, 125000, 7, 1, 8, false, true, false, 0, false);

    sx1276_set_rx_config(dev, MODEM_LORA, 125000, 7, 1, 0, 8, 5, false, 0, true, false, 0, false, true);
    char some_arr[] = "hello\n";
    #if 0
#endif
int cnt=0;
    while(cnt < 4000) {
        cnt = cnt + 1;
        //k_busy_wait(1000000);
	
        sx1276_send_str(dev, some_arr, 3000);
        sx1276_set_sleep(dev);
printf("cnt value: %d\n", cnt);
upm_delay_ms(1000);
    }


    printf("coming through\n");
/////////////////////////////////////////////////////////////////
//k_sem_init(&a_sem, 0, 1);
//k_sem_init(&b_sem, 1, 1);
	//k_thread_spawn(thread_stack_area, 1024, thread_interrupt, dev, NULL, NULL,
	//	       1, 0, K_NO_WAIT);
	//k_thread_spawn(thread_stack_area, 1024, thread_functions, dev, NULL, NULL,
	//	       1, 0, K_NO_WAIT);
/////////////////////////////////////////////////////////////////
#if 0
    if(sx1276_set_channel(dev, 915000000) != UPM_SUCCESS) {
        printf("issues setting the channel\n");
    }

    sx1276_set_tx_config(dev, MODEM_LORA, 14, 0, 125000, 7, 1, 8, false, true, false, 0, false);

    sx1276_set_rx_config(dev, MODEM_LORA, 125000, 7, 1, 0, 8, 5, false, 0, true, false, 0, false, true);
    char some_arr[] = "hello\n";

    //while(1) {
        sx1276_send_str(dev, some_arr, 3000);
        //k_sleep(1);
    //}

    printf("coming through\n");
#endif
    return;
}

#if 0
void thread_interrupt(void *dummy1, void *dummy2, void *dummy3)
{
//k_sem_take(&my_sem, K_FOREVER);
printf("hello from the thread\n");
    sx1276_context dev = (sx1276_context) dummy1;
    handle_interrupts(dev);

}
#endif

#if 1

void thread_functions(void *dummy1, void *dummy2, void *dummy3)
{

//k_sem_take(&my_sem, K_FOREVER);


}
#endif
*/
