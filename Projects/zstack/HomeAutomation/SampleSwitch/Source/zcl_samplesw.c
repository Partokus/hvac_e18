/**************************************************************************************************
  Filename:       zcl_samplesw.c
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $

  Description:    Zigbee Cluster Library - sample switch application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee On/Off Switch, based on Z-Stack 3.0.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample applicetion.
  
  Application-specific UI peripherals being used:

  - none (LED1 is currently unused by this application).

  Application-specific menu system:

    <TOGGLE LIGHT> Send an On, Off or Toggle command targeting appropriate devices from the binding table.
      Pressing / releasing [OK] will have the following functionality, depending on the value of the 
      zclSampleSw_OnOffSwitchActions attribute:
      - OnOffSwitchActions == 0: pressing [OK] will send ON command, releasing it will send OFF command;
      - OnOffSwitchActions == 1: pressing [OK] will send OFF command, releasing it will send ON command;
      - OnOffSwitchActions == 2: pressing [OK] will send TOGGLE command, releasing it will not send any command.

*********************************************************************/

#if ! defined ZCL_ON_OFF
#error ZCL_ON_OFF must be defined for this project.
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"


#include "OSAL_PwrMgr.h"
#include "hal_drivers.h"


#include "hal_uart.h"
//#include "_hal_uart_isr.c"

#include "hal_sleep.h"
#include "OSAL_PwrMgr.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_samplesw.h"
#include "zcl_diagnostic.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#include "zcl_ota.h"
#include "hal_ota.h"
#endif

#include "bdb.h"
#include "bdb_interface.h"

#include "zcl_sampleapps_ui.h"

   
/* -------------------- */
   #include "hal_types.h"

/* high-level */
#include "mac_pib.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_radio.h"
#include "mac_tx.h"
#include "mac_rx.h"
#include "mac_rx_onoff.h"
#include "mac_sleep.h"
#include "mac_backoff_timer.h"

/* target specific */
#include "mac_radio_defs.h"

/* debug */
#include "mac_assert.h"
/* -------------------- */
/*********************************************************************
 * MACROS
 */
#define UI_STATE_TOGGLE_LIGHT 1 //UI_STATE_BACK_FROM_APP_MENU is item #0, so app item numbers should start from 1

#define APP_TITLE "TI Sample Switch"



/* Version data */
#define REMOTE_HEADER 0x02

#define VERSION_HEADER 0x06

#define VERSION_MAJOR 0
#define VERSION_MIDDLE 2
#define VERSION_MINOR 0


// firmware version format packet: {0x70, 0x65, 0x00, 0x00, Number_of_data_bytes (from version_header to minor), version_header, type_of_device, major, middle, minor, LRC_of_data_bytes}
static uint8 firmware_version[11] = {0x70, 0x65, 0x00, 0x00, 0x05, VERSION_HEADER, REMOTE_HEADER, VERSION_MAJOR, VERSION_MIDDLE, VERSION_MINOR, 0x00}; // Coordinator_conditioner's version packet

static bool is_first_send_firmware_version = true;

static uint8 MySerialApp_CalcLrc(uint8 *data, size_t dataSize);
/* Version data end */




#if (HAL_UART_DMA == 1) || (HAL_UART_ISR == 1)
#define MY_SERIAL_APP_SERIAL_0
#endif
#if (HAL_UART_DMA == 2) || (HAL_UART_ISR == 2)
//#define MY_SERIAL_APP_SERIAL_1
#endif

#if !defined( MY_SERIAL_APP_PORT_0 )
#define MY_SERIAL_APP_PORT_0  0
#endif

#if !defined( MY_SERIAL_APP_PORT_1 )
#define MY_SERIAL_APP_PORT_1  1
#endif

#if !defined( MY_SERIAL_APP_BAUD )
#define MY_SERIAL_APP_BAUD  HAL_UART_BR_9600
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( MY_SERIAL_APP_THRESH )
#define MY_SERIAL_APP_THRESH  64
#endif

#if !defined( MY_SERIAL_APP_RX_SZ )
#define MY_SERIAL_APP_RX_SZ  128 // 128
#endif

#if !defined( MY_SERIAL_APP_TX_SZ )
#define MY_SERIAL_APP_TX_SZ  128 // 128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( MY_SERIAL_APP_IDLE )
#define MY_SERIAL_APP_IDLE  6
#endif

// Echo Rx bytes to Tx for throughput testing.
#if !defined( MY_SERIAL_APP_ECHO )
#define MY_SERIAL_APP_ECHO  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( MY_SERIAL_APP_TX_MAX )
#define MY_SERIAL_APP_TX_MAX  80 // 80
#endif   
   
   
#if defined(MY_SERIAL_APP_SERIAL_0)
static uint8 MySerialApp_Tx0Buf[MY_SERIAL_APP_TX_MAX];
static uint8 MySerialApp_Tx0Len;
#endif

#if defined(MY_SERIAL_APP_SERIAL_1)
static uint8 MySerialApp_Tx1Buf[MY_SERIAL_APP_TX_MAX];
static uint8 MySerialApp_Tx1Len;
#endif
   
   
   
  
   
#if ZG_BUILD_JOINING_TYPE
static uint16 MySerialApp_LastPanId;
//static void MySerialApp_FilterNwkDesc( networkDesc_t *pBDBListNwk, uint8 count );
//static void MySerialApp_SortNetworkDesc( networkDesc_t* desc );
//static void MySerialApp_RollNetworkDesc( networkDesc_t* desc, uint16 lastPanId );
#endif 

#define uart_wake_up_interrupt_enable() P0IFG = 0; P0IF = 0; P0IEN |= BV(2)
#define uart_wake_up_interrupt_disable() P0IFG = 0; P0IF = 0; P0IEN &= ~BV(2)

#define UxCSR                      U0CSR
#define CSR_RE                     0x40
#define UTX0IE                     0x04

#define NV_SERIALAPP_LAST_PANID     0x0402
   

uint8 debug_temp_1 = 8;
uint8 debug_temp_2 = 8;

/* COMMANDS */
const uint8 headerArr[] = {
  0x02, 0xA8, 0x79, 0xC3
};

enum
{
  QUERY_NETWORK_STATUS      = 1,
  RESTORE_FACTORY_SETTINGS  = 3,
  SET_PANID                 = 4,
  READ_PANID                = 5,
  MY_SYSTEM_RESET           = 6,
  SET_POLL_RATE             = 7
};

#define NUM_OF_ELEMENTS(x) ( sizeof(x) / sizeof(x[0]) )

const uint8 headerSize = NUM_OF_ELEMENTS( headerArr );
devStates_t zclSampleLight_NwkState;
//bool ATMEEX_rejoin_processing = 0;


static bool checkHeader( uint8* pBuf, uint8 len );
static void leaveNetwork( void );
static void queryNetworkStatus( uint8 port );
static void readPANID( uint8 port );
static void setPANID( uint8 port, uint16 PANID );
static bool processCommand( uint8* pBuf, uint8 len, uint8 port );
//static void PortMapUart( uint8 PortAlt, uint8 PortNum );
static void Lepeshkin_InterruptForUartInit( void );
static void atmeex_keys_init( void );
static bool led_stat = 0;
void atmeex_led_on( void );
void atmeex_led_off( void );
void atmeex_led_toggle( void );
static void atmeex_init( void );
//static void ATMEEX_send_test_data( void );
static void ATMEEX_POWER_SAVING_MODE_TURN_ON( void );
static void ATMEEX_POWER_SAVING_MODE_TURN_OFF( uint8 cMask );
#if defined(MY_SERIAL_APP_SERIAL_0)
static void ATMEEX_send_data_over_air0( void );
static void MySerialApp_CallBack0( uint8 port, uint8 event );
#endif
#if defined(MY_SERIAL_APP_SERIAL_1)
static void ATMEEX_send_data_over_air1( void );
static void MySerialApp_CallBack1( uint8 port, uint8 event );
#endif 
//static void ATMEEX_BindNotificationCB( bdbBindNotificationData_t *data );
uint8 atmeex_mask = 0;
bool UART_AVAIL = 1;
bool ATMEEX_OTA_FLAG = 0; // flag if we are processing OTA for AVR
uint8 can_send_data = 1;
uint16 atmeex_firmware_number = 1; 

bool atmeex_TX_empty = 1; // 1 - empty, 0 - no


void SendData( afAddrType_t *DstAddrPointer, uint8 cmd, uint8 dataBufLen, uint8 *dataBufPointer );

static void my_set_poll_rate(uint8 my_poll_rate_t);
static uint32 my_poll_rate = SEND_GET_REQUEST_DEFAULT_PERIOD;





/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleSw_TaskID;

uint8 zclSampleSwSeqNum = 0;

uint8 zclSampleSw_OnOffSwitchType = ON_OFF_SWITCH_TYPE_MOMENTARY;

uint8 zclSampleSw_OnOffSwitchActions;



/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleSw_DstAddr;
afAddrType_t zclSampleSw_Groupcast_DstAddr;

// Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleSw_TestEp =
{
  SAMPLESW_ENDPOINT,                  // endpoint
  0,
  &zclSampleSw_TaskID,
  (SimpleDescriptionFormat_t *)&zclSampleSw_SimpleDesc,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

//static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

devStates_t zclSampleSw_NwkState = DEV_INIT;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#define DEVICE_POLL_RATE                 8000   // Poll rate for end device
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void zclSampleSw_HandleKeys( byte shift, byte keys );
//static void zclSampleSw_BasicResetCB( void );
static void zclSampleLight_OnOffCB( uint8 cmd ); // atmeex


static void zclSampleSw_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);


// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
static void zclSampleSw_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg );
#endif

//void zclSampleSw_UiActionToggleLight(uint16 keys);
//void zclSampleSw_UiUpdateLcd(uint8 uiCurrentState, char * line[3]);

//static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);



  
/*********************************************************************
 * REFERENCED EXTERNALS
 */
extern int16 zdpExternalStateTaskID;

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleSw_CmdCallbacks =
{
  NULL, //zclSampleSw_BasicResetCB,               // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  zclSampleLight_OnOffCB,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleSw_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleSw_Init( byte task_id )
{
  zclSampleSw_TaskID = task_id;
  
  // Set destination address to Groupcast
  zclSampleSw_DstAddr.addrMode = (afAddrMode_t)AddrGroup;
  zclSampleSw_DstAddr.endPoint = 8;
  zclSampleSw_DstAddr.addr.shortAddr = 0x000A;
  
  // Set destination address to Broadcast
  //zclSampleSw_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  //zclSampleSw_DstAddr.endPoint = 8;
  //zclSampleSw_DstAddr.addr.shortAddr = 0xFFFF;

  //zclSampleSw_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  //zclSampleSw_DstAddr.endPoint = 0;
  //zclSampleSw_DstAddr.addr.shortAddr = 0;
  
  atmeex_init();
  
  
  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &zclSampleSw_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLESW_ENDPOINT, &zclSampleSw_CmdCallbacks );

  zclSampleSw_ResetAttributesToDefaultValues();
  
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLESW_ENDPOINT, zclSampleSw_NumAttributes, zclSampleSw_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleSw_TaskID );
  
  // Register low voltage NV memory protection application callback
  //RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleSw_TaskID );
  
  bdb_RegisterCommissioningStatusCB( zclSampleSw_ProcessCommissioningStatus );
  //bdb_RegisterBindNotificationCB(ATMEEX_BindNotificationCB);
  // Register for a test endpoint
  afRegister( &sampleSw_TestEp );
  
#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SAMPLESW_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
  // Register for callback events from the ZCL OTA
  zclOTA_Register(zclSampleSw_TaskID);
#endif

  zdpExternalStateTaskID = zclSampleSw_TaskID;
  //P1_1 = bdb_isDeviceNonFactoryNew();
#if (ZG_BUILD_COORDINATOR_TYPE)
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_FINDING_BINDING);
#else
  
  
  
  
  uint8 nv_status1 = osal_nv_item_init( NV_SERIALAPP_LAST_PANID, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId );
  if (nv_status1 == SUCCESS) // if Id already existed, no action taken.
  {
    osal_nv_read( NV_SERIALAPP_LAST_PANID, 0, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId ); // read inter pan id
  } 
  
  //bdb_RegisterForFilterNwkDescCB( MySerialApp_FilterNwkDesc );
  
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING /* | BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_FINDING_BINDING */);
  
#endif
  //osal_start_timerEx( zclSampleSw_TaskID, ATMEEX_FIRST_JOIN_TO_NETWORK, 20000 );

}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleSw_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  
  //Send toggle every 500ms  
  if( events & SAMPLESW_TOGGLE_TEST_EVT )
  {
    can_send_data = 1;
    return (events ^ SAMPLESW_TOGGLE_TEST_EVT);
  }
  
  
  
  if ( events & SYS_EVENT_MSG )
  {
    
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleSw_TaskID )) )
    {
      
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleSw_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          
          break;

        case KEY_CHANGE:
          //zclSampleSw_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleLight_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (zclSampleLight_NwkState == DEV_ZB_COORD) ||
          (zclSampleLight_NwkState == DEV_ROUTER) ||
          (zclSampleLight_NwkState == DEV_END_DEVICE) )
          {
            if (is_first_send_firmware_version) 
            {
              osal_start_timerEx( zclSampleSw_TaskID, SEND_VERSION_EVT, SEND_VERSION_EVT_DELAY );
            }  
            
            osal_set_event( zclSampleSw_TaskID, SEND_GET_REQUEST);
            NLME_SetPollRate(3000); // set poll rate 3 sec temporary
            osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_SET_DEFAULT_POLL_RATE_ON_START, 15000); // set default poll rate
          }
          break;
        

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
        case ZCL_OTA_CALLBACK_IND:
          zclSampleSw_ProcessOTAMsgs( (zclOTA_CallbackMsg_t*)MSGpkt  );
          break;
#endif

        default:
          break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  
  
  
  
#if defined(MY_SERIAL_APP_SERIAL_0)
  if ( events & ATMEEX_UART_DATA_READY_SERIAL0 )
  { 
    osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_POWER_SAVING_MODE_ON, DELAY_BEFORE_SLEEP);
    atmeex_clear_sleep_mask(INTERRUPT_WAKE_UP_MASK);
    
    if ( processCommand( MySerialApp_Tx0Buf, MySerialApp_Tx0Len, MY_SERIAL_APP_PORT_0 ) )
    {
      MySerialApp_Tx0Len = 0;  
      
    } else
    {
      //atmeex_led_toggle();
      ATMEEX_send_data_over_air0();
    }
    
    //ATMEEX_POWER_SAVING_MODE_TURN_ON(); // turn on power saving mode
    // return unprocessed events
    return ( events ^ ATMEEX_UART_DATA_READY_SERIAL0 );
  }
  
  if ( events & ATMEEX_POWER_SAVING_MODE_ON )
  {  

    if (atmeex_TX_empty == 1) // if not sending data throught UART right now
    {
      ATMEEX_POWER_SAVING_MODE_TURN_ON(); // turn on power saving mode
    } else // try again
    {
      osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_POWER_SAVING_MODE_ON, 20); 
      //atmeex_led_toggle();
    }
    
    return ( events ^ ATMEEX_POWER_SAVING_MODE_ON );
  }
#endif
  
#if defined(MY_SERIAL_APP_SERIAL_1)
  
  if ( events & ATMEEX_UART_DATA_READY_SERIAL1 )
  {
    if ( processCommand( MySerialApp_Tx1Buf, MySerialApp_Tx1Len, MY_SERIAL_APP_PORT_1 ) )
    {
      MySerialApp_Tx1Len = 0;
    }
    else
    {
      ATMEEX_send_data_over_air1();
    }
    // return unprocessed events
    return ( events ^ ATMEEX_UART_DATA_READY_SERIAL1 );
  }
#endif
  
    

  
  
#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
  {
    //atmeex_led_toggle();
    bdb_ZedAttemptRecoverNwk();
    
    return ( events ^ SAMPLEAPP_END_DEVICE_REJOIN_EVT );
  }
#endif
  
  
  
  if ( events & ATMEEX_SET_DEFAULT_POLL_RATE_ON_START )
  {
    NLME_SetPollRate(POLL_RATE); // set default poll rate
    return ( events ^ ATMEEX_SET_DEFAULT_POLL_RATE_ON_START );
  }
  
  
  
  if ( events & SEND_GET_REQUEST )
  {
    SendData( &zclSampleSw_DstAddr, COMMAND_GET, 0, 0 ); 

    osal_start_timerEx( zclSampleSw_TaskID, SEND_GET_REQUEST, my_poll_rate );
    return ( events ^ SEND_GET_REQUEST );
  }
  




  
  
  
  
  
  
  
  
  
  
  // stop OTA for AVR, return default parameters
  if( events & STOP_OTA_FOR_AVR_TIMER )
  {
    ATMEEX_OTA_FLAG = 0;
    atmeex_clear_sleep_mask(OTA_FOR_AVR_MASK); // enable sleep mode
    // Lower up the poll rate
    uint8 RxOnIdle;
    RxOnIdle = FALSE;
    ZMacSetReq( ZMacRxOnIdle, &RxOnIdle ); 
    NLME_SetPollRate( POLL_RATE ); // return default poll rate 
    //osal_set_event(zclSampleSw_TaskID, SEND_GET_REQUEST); // start making get request every 10 sec      
    osal_start_timerEx( zclSampleSw_TaskID, ATMEEX_POWER_SAVING_MODE_ON, DELAY_BEFORE_SLEEP);   
    return (events ^ STOP_OTA_FOR_AVR_TIMER);
  }
 
  if ( events & SEND_VERSION_EVT )
  {
    uint8 lrc_t = MySerialApp_CalcLrc(&firmware_version[5], sizeof(firmware_version) - 6);
    firmware_version[sizeof(firmware_version) - 1] = lrc_t;
    
    SendData( &zclSampleSw_DstAddr, COMMAND_SEND, sizeof(firmware_version), firmware_version );

    return ( events ^ SEND_VERSION_EVT );
  }
  
  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleSw_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
/*
static void zclSampleSw_HandleKeys( byte shift, byte keys )
{
  //UI_MainStateMachine(keys);
}
*/


/*********************************************************************
 * @fn      zclSampleSw_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleSw_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been processed yet
        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
        
#if ZG_BUILD_JOINING_TYPE
        
        if ( MySerialApp_LastPanId != _NIB.nwkPanId )
        {
          //MySerialApp_LastPanId = _NIB.nwkPanId;
          //osal_nv_write( NV_SERIALAPP_LAST_PANID, 0, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId );
          //Forbid_or_allow_power_saving_mode(1); // allow power saving mode
        }
#endif
      }
      else
      {
        //See the possible errors for nwk steering procedure
        //No suitable networks found
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization 
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
      
      //YOUR JOB:
      //We are on a network, what now?
      
    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
    //    short_or_default_delay_counter = 0; // set default value
        //osal_set_event( zclSampleSw_TaskID, SEND_GET_REQUEST);
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        //osal_stop_timerEx( zclSampleSw_TaskID, SEND_GET_REQUEST);
        
       // if (short_or_default_delay_counter < 10)
       // {
       //   osal_start_timerEx(zclSampleSw_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_SHORT_DELAY);
       //   short_or_default_delay_counter++;
       // } else {
          osal_start_timerEx(zclSampleSw_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);         
      //  } 
        
        //osal_start_timerEx(zclSampleSw_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);   
      }
    break;
#endif 
  }
  
  //UI_UpdateComissioningStatus(bdbCommissioningModeMsg);
}

/*********************************************************************
 * @fn      zclSampleSw_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
/*
static void zclSampleSw_BasicResetCB( void )
{
  zclSampleSw_ResetAttributesToDefaultValues();

  // update the display
  //UI_UpdateLcd( ); 
}
*/

/*********************************************************************
 * @fn      zclSampleApp_BatteryWarningCB
 *
 * @brief   Called to handle battery-low situation.
 *
 * @param   voltLevel - level of severity
 *
 * @return  none
 */
/*
void zclSampleApp_BatteryWarningCB( uint8 voltLevel )
{
  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
  {
    // Send warning message to the gateway and blink LED
  }
  else if ( voltLevel == VOLT_LEVEL_BAD )
  {
    // Shut down the system
  }
}
*/

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleSw_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleSw_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleSw_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleSw_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleSw_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleSw_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleSw_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleSw_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleSw_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleSw_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleSw_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleSw_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleSw_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
/*********************************************************************
 * @fn      zclSampleSw_ProcessOTAMsgs
 *
 * @brief   Called to process callbacks from the ZCL OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleSw_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg )
{
  uint8 RxOnIdle;

  switch(pMsg->ota_event)
  {
  case ZCL_OTA_START_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Speed up the poll rate
      RxOnIdle = TRUE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate( 2000 );
    }
    break;

  case ZCL_OTA_DL_COMPLETE_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Reset the CRC Shadow and reboot.  The bootloader will see the
      // CRC shadow has been cleared and switch to the new image
      HalOTAInvRC();
      SystemReset();
    }
    else
    {
#if (ZG_BUILD_ENDDEVICE_TYPE)    
      // slow the poll rate back down.
      RxOnIdle = FALSE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate(DEVICE_POLL_RATE);
#endif
    }
    break;

  default:
    break;
  }
}
#endif // defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)

/****************************************************************************
****************************************************************************/
/*
void zclSampleSw_UiActionToggleLight(uint16 keys)
{
  
  if (zclSampleSw_OnOffSwitchActions == ON_OFF_SWITCH_ACTIONS_TOGGLE)
  {
    if (keys & UI_KEY_SW_5_PRESSED)
    {
      zclGeneral_SendOnOff_CmdToggle( SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, FALSE, bdb_getZCLFrameCounter() );
    }
  }
  else if (((keys & UI_KEY_SW_5_PRESSED) && (zclSampleSw_OnOffSwitchActions == ON_OFF_SWITCH_ACTIONS_ON))
    || ((keys & UI_KEY_SW_5_RELEASED) && (zclSampleSw_OnOffSwitchActions == ON_OFF_SWITCH_ACTIONS_OFF)))
  {
    zclGeneral_SendOnOff_CmdOn( SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, FALSE, bdb_getZCLFrameCounter() );
  }
  else
  {
    zclGeneral_SendOnOff_CmdOff( SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, FALSE, bdb_getZCLFrameCounter() );
  }

}
*/
/*
void zclSampleSw_UiUpdateLcd(uint8 gui_state, char * line[3])
{
  //line[2] = "< TOGGLE LIGHT >";
}
*/


static void atmeex_keys_init( void )
{
   #if ZG_BUILD_ENDDEVICE_TYPE 
   P1SEL &= ~BV(1); // gpio func
   P1DIR |= BV(1); // output
   P1_1 = 0;
   #else
   P0SEL &= ~BV(1); // gpio func
   P0DIR |= BV(1); // output
   P0_6 = 0;       
   #endif
}



void atmeex_led_on( void )
{
  #if ZG_BUILD_ENDDEVICE_TYPE 
   P1_1 = 1;
   #else
   P0_6 = 1;       
   #endif
}

void atmeex_led_off( void )
{
  #if ZG_BUILD_ENDDEVICE_TYPE 
   P1_1 = 0;
   #else
   P0_6 = 0;       
   #endif
}

void atmeex_led_toggle( void )
{
  led_stat = !led_stat;
  
  #if ZG_BUILD_ENDDEVICE_TYPE 
  P1_1 = led_stat;
  #else
  P0_6 = led_stat;       
  #endif
}



static void Lepeshkin_InterruptForUartInit( void ) {
  /* For interrupt by falling edge, the bit must be set. */
  P0SEL &= ~(1 << 2); // P0_2 GPIO function
  P0DIR &= ~(1 << 2); // P0_2 GPIO input mode
  //P0_2 = 1; // pull-up
  PICTL |= BV(2); // interrupt by FALLING edge on RX pin (P0_2)
  //PICTL &= ~BV(2); // interrupt by RISING edge on RX pin (P0_2)
  
  /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
  P0IEN |= BV(2); // enable/disable interrupt
  //P0IEN &= ~BV(2);
  IEN1 |= BV(5); // turn on CPU interrupt
  P0IFG = ~(BV(2)); // clear interrupt flag
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();
 
  if (P0IFG & BV(2))
  {
    P0IFG = ~(BV(2)); /* Clear Interrupt Flag */
    //led_stat = !led_stat;
    //P1_1 = led_stat;
    //P1_1 = 1;

    
    ATMEEX_POWER_SAVING_MODE_TURN_OFF(INTERRUPT_WAKE_UP_MASK);
    //uart_wake_up_interrupt_disable();
    //osal_pwrmgr_device(PWRMGR_ALWAYS_ON); //PWRMGR_ALWAYS_ON PWRMGR_BATTERY
    if ((UxCSR & CSR_RE) == 0) // if UART turned off
    {
      halSleepWait(1500); // 1.5 ms
      HalUARTResume(); // Turn on UART
    }  
  } 
  
  
  
  
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

/*
static void atmeex_zcl_sendReportCmd( void )
{
  zclReportCmd_t *pReportCmd;
  
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_ON_OFF;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_BOOLEAN;
    pReportCmd->attrList[0].attrData = (void *)(&zclSampleLight_OnOff);


    zcl_SendReportCmd(SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF,
                                     pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zcl_seqNum++ );
  }

  osal_mem_free( pReportCmd );
}
*/


















/* ATMEEX: data got callback */
/*********************************************************************
 * @fn      zclSampleLight_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none

 * pPtr->cmd.Data[0] and pPtr->cmd.Data[1] and pPtr->cmd.Data[2] unknown data
 * pPtr->cmd.Data[3] - first byte in the sending array ([0])
 * pPtr->cmd.Data[LASTBYTE] - data lenght
  //zclSampleSw_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
 */
static void zclSampleLight_OnOffCB( uint8 cmd )
{
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();
  
  
  if (NLME_GetShortAddr() ==  pPtr->srcAddr.addr.shortAddr) // if it own address
  {
    // Release the memory
    //osal_msg_deallocate( (uint8 *)pPtr );
    return; 
  }
  
  
  
  
    if (pPtr->cmd.Data[3] == 0x02 && pPtr->cmd.Data[7] != 0x45) // if it OTA format packet and it is not firmaware's version check command
    {
      //atmeex_led_toggle();
      
      osal_start_timerEx(zclSampleSw_TaskID, STOP_OTA_FOR_AVR_TIMER, OTA_STOP_DELAY); // start timer for stopping OTA for AVR if OTA packet will not come for OTA_STOP_DELAY milliseconds

      if (ATMEEX_OTA_FLAG == 0) // if we are first time get OTA packet format, so start OTA
      {
      ATMEEX_POWER_SAVING_MODE_TURN_OFF( OTA_FOR_AVR_MASK ); // permit sleep.
      // Speed up the poll rate
      uint8 RxOnIdle;
      RxOnIdle = TRUE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle ); 
      NLME_SetPollRate( 2000 ); // make poll rate faster
      osal_stop_timerEx(zclSampleSw_TaskID, ATMEEX_SET_DEFAULT_POLL_RATE_ON_START); // stop timer for default poll rate
      //osal_stop_timerEx(zclSampleSw_TaskID, SEND_GET_REQUEST); // stop making get request every 10 sec      
      ATMEEX_OTA_FLAG = 1;
      }
    } else
    {
      if (pPtr->cmd.Data[8] == 0x02) // if it command from Mobile App
      {
        
        osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_SET_DEFAULT_POLL_RATE_ON_START, POLL_RATE_FOR_MOBILE_APP_DELAY); // set default poll rate later
        NLME_SetPollRate( 3000 ); // make poll rate faster
        
      }
    }
    
  
  
  
  
  
    CLEAR_SLEEP_MODE();   
    ATMEEX_POWER_SAVING_MODE_TURN_OFF( 0x00 ); // permit sleep.
  
    
    if ((UxCSR & CSR_RE) == 0) // if UART turned off
    {
      HalUARTResume(); // Turn on UART
    }   
    
    osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_POWER_SAVING_MODE_ON, DELAY_BEFORE_SLEEP);   
    
    
    
    
    
    
    if (atmeex_TX_empty == 1)
    {
      atmeex_TX_empty = 0;
#if defined(MY_SERIAL_APP_SERIAL_0)
    HalUARTWrite( MY_SERIAL_APP_PORT_0, &(pPtr->cmd.Data[3]), (pPtr->cmd.DataLength - 3) ); // 0,1,2 byte - not TMEEX's application data
#endif
    
#if defined(MY_SERIAL_APP_SERIAL_1)
    HalUARTWrite( MY_SERIAL_APP_PORT_1, &(pPtr->cmd.Data[3]), (pPtr->cmd.DataLength - 3) ); // 0,1,2 byte - not TMEEX's application data 
#endif  
    }
    
    
    
    
    
    // Release the memory
    //osal_msg_deallocate( (uint8 *)pPtr );
}

















static void atmeex_init( void )
{
  atmeex_keys_init();
  Lepeshkin_InterruptForUartInit();
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = MY_SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = MY_SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = MY_SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = MY_SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = MY_SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.

#if defined(MY_SERIAL_APP_SERIAL_0)
  uartConfig.callBackFunc         = MySerialApp_CallBack0;
  HalUARTOpen( MY_SERIAL_APP_PORT_0, &uartConfig );
#endif

#if defined(MY_SERIAL_APP_SERIAL_1)
  uartConfig.callBackFunc         = MySerialApp_CallBack1;
 
  HalUARTOpen( MY_SERIAL_APP_PORT_1, &uartConfig );
  PortMapUart( 1, MY_SERIAL_APP_PORT_1 );
#endif
  
  /*
  // add yourself in group
  aps_Group_t group;
  // Assign yourself to group A
  group.ID = 0x000A;
  group.name[0] = 6; // First byte is string length
  osal_memcpy( &(group.name[1]), "GroupA", 6);
  aps_AddGroup( SAMPLESW_ENDPOINT, &group );
  */
}
















#if defined(MY_SERIAL_APP_SERIAL_0)

/*********************************************************************
 * @fn      MySerialApp_CallBack0
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void MySerialApp_CallBack0( uint8 port, uint8 event )
{
  
  (void)port;
  if (event & HAL_UART_TX_EMPTY) {
      atmeex_TX_empty = 1;
  }
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
      (MySerialApp_Tx0Len < MY_SERIAL_APP_TX_MAX))
  {
    MySerialApp_Tx0Len += HalUARTRead(MY_SERIAL_APP_PORT_0, MySerialApp_Tx0Buf + MySerialApp_Tx0Len,
                                      MY_SERIAL_APP_TX_MAX - MySerialApp_Tx0Len);

    
    osal_start_timerEx( zclSampleSw_TaskID, ATMEEX_UART_DATA_READY_SERIAL0, 50 );  
  }  
  

  
}
#endif // MY_SERIAL_APP_SERIAL_0

#if defined(MY_SERIAL_APP_SERIAL_1)

static void MySerialApp_CallBack1( uint8 port, uint8 event )
{
  (void)port;
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
      (MySerialApp_Tx1Len < MY_SERIAL_APP_TX_MAX))
  {
    MySerialApp_Tx1Len += HalUARTRead(MY_SERIAL_APP_PORT_1, MySerialApp_Tx1Buf + MySerialApp_Tx1Len,
                                      MY_SERIAL_APP_TX_MAX - MySerialApp_Tx1Len);
    
    osal_start_timerEx( zclSampleSw_TaskID, ATMEEX_POWER_SAVING_MODE_ON, 80);
    osal_start_timerEx( zclSampleSw_TaskID, ATMEEX_UART_DATA_READY_SERIAL1, 50 );
  }
 // if (event & HAL_UART_TX_EMPTY) {
 //   ATMEEX_UART_AVAIL = 1; 
 // }
  
  
}

#endif // MY_SERIAL_APP_SERIAL_1



static bool processCommand( uint8* pBuf, uint8 len, uint8 port )
{
  if ( len < headerSize + 1 )
    return FALSE;
  
  if ( !checkHeader(pBuf, len) )
    return FALSE;
  
  switch ( pBuf[headerSize] )
  {
  case QUERY_NETWORK_STATUS:
    queryNetworkStatus(port);
    break;
    
  case RESTORE_FACTORY_SETTINGS:
    break;
    
  case SET_PANID:
    //if (len > headerSize + 2)
    //{
      setPANID(port, pBuf[headerSize + 1] << 8 | pBuf[headerSize + 2]);
    //}
    break;
    
  case READ_PANID:
    readPANID(port);
    break;
  
  case MY_SYSTEM_RESET:
    SystemReset();
    break;
    
  case SET_POLL_RATE:
    my_set_poll_rate(pBuf[headerSize + 1]);
    break;
    
  default:
    return FALSE;
  }
    
  return TRUE;
}

#if defined(MY_SERIAL_APP_SERIAL_1)
static void PortMapUart( uint8 PortAlt, uint8 PortNum )
{
  // If UART Port Alternative 1 desired
  if( PortAlt == 1 )
  {
    // If UART0 desired
    if ( PortNum == 0 )
    {
      // Configure UART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
      PERCFG &= ~0x01;
      // Configure relevant Port P0 pins for peripheral function:
      // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
      P0SEL |= 0x3C;
      // Configure relevant Port P1 pins back to GPIO function
      P1SEL &= ~0x3C;
      // Else (UART1 desired)
    }
    else
    {
      // Configure UART1 for Alternative 1 => Port P0 (PERCFG.U1CFG = 0)
      PERCFG &= ~0x02;
      // Configure relevant Port P0 pins for peripheral function:
      // P0SEL.SELP0_2/3/4/5 = 1 => CT = P0_2, RT = P0_3, TX = P0_4, RX = P0_5
      P0SEL |= 0x3C;
      // Configure relevant Port P1 pins back to GPIO function
      P1SEL &= ~0xF0;
    }
    // Else (UART Port Alternative 2 desired)
  }
  else
  {
    // If UART0 desired
    if ( PortNum == 0 )
    {
      // Configure UART0 for Alternative 2 => Port P1 (PERCFG.U0CFG = 1)
      PERCFG |= 0x01;
      // P1SEL.SELP1_2/3/4/5 = 1 => CT = P1_2, RT = P1_3, RX = P1_4, TX = P1_5
      P1SEL |= 0x3C;
      // Configure relevant Port P0 pins back to GPIO function
      P0SEL &= ~0x3C;
      // Else (UART1 desired)
    }
    else
    {
      // Configure UART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
      PERCFG |= 0x02;
      // P1SEL.SELP1_4/5/6/7 = 1 => CT = P1_4, RT = P1_5, TX = P1_6, RX = P1_7
      P1SEL |= 0xF0;
      // Configure relevant Port P0 pins back to GPIO function
      P0SEL &= ~0x3C;
    }
  }
}
#endif

static bool checkHeader( uint8* pBuf, uint8 len )
{
  if ( len < headerSize )
    return FALSE;
  
  for ( uint8 i = 0; i < headerSize; ++i )
  {
    if ( pBuf[i] != headerArr[i] )
      return FALSE;
  }
  
  return TRUE;
}


static void leaveNetwork( void )
{
  zgWriteStartupOptions( ZG_STARTUP_SET, (ZCD_STARTOPT_DEFAULT_NETWORK_STATE | ZCD_STARTOPT_DEFAULT_CONFIG_STATE) );
  SystemReset();
  /*
  NLME_LeaveReq_t leaveReq;
  // Set every field to 0
  osal_memset( &leaveReq, 0, sizeof( NLME_LeaveReq_t ) );

  // This will enable the device to rejoin the network after reset.
  leaveReq.rejoin = FALSE;
  
  // Set the NV startup option to force a "new" join.
  zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE | ZCD_STARTOPT_DEFAULT_CONFIG_STATE );

  // Leave the network, and reset afterwards
  if ( NLME_LeaveReq(&leaveReq) != ZSuccess )
  {
    // Couldn't send out leave; prepare to reset anyway
    ZDApp_LeaveReset(FALSE);
  }
  */
}


static void queryNetworkStatus( uint8 port )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0 };
  answer[5] = devState;

  if (atmeex_TX_empty == 1)
    {
      atmeex_TX_empty = 0;
  HalUARTWrite( port, answer, NUM_OF_ELEMENTS( answer ) );
    }
}

static void readPANID( uint8 port )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  uint16 PANID = _NIB.nwkPanId;
  answer[5] = PANID >> 8;
  answer[6] = PANID;

  if (atmeex_TX_empty == 1)
    {
      atmeex_TX_empty = 0;
      HalUARTWrite(port, answer, NUM_OF_ELEMENTS(answer));
    }
}

static void setPANID( uint8 port, uint16 PANID )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0 };
  leaveNetwork();
  //bdb_resetLocalAction();
  answer[4] = 0x4F;
  answer[5] = 0x4B;
  if (atmeex_TX_empty == 1)
    {
      atmeex_TX_empty = 0;
  HalUARTWrite(port, answer, NUM_OF_ELEMENTS(answer));
    }
}

static void my_set_poll_rate(uint8 my_poll_rate_t)
{
  my_poll_rate = my_poll_rate_t * 1000;
}

/*
static void ATMEEX_send_test_data( void )
{
  uint8 atmeex_message[25] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};     
  //zcl_SendCommand(SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF, COMMAND_TOGGLE, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, FALSE, 0, zcl_seqNum, 25, atmeex_message);
  zcl_
(SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF, COMMAND_ON, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, 0, 0, 25, atmeex_message);
}
*/

static void ATMEEX_POWER_SAVING_MODE_TURN_ON( void )
{
  if (!atmeex_mask)
  {
   //atmeex_led_off();
   osal_pwrmgr_device(PWRMGR_BATTERY); // PWRMGR_ALWAYS_ON PWRMGR_BATTERY
   uart_wake_up_interrupt_enable();    // enable interrupt for RISING EDGE on RX pin of UART
  }
}


static void ATMEEX_POWER_SAVING_MODE_TURN_OFF( uint8 cMask )
{
  //atmeex_led_on();
  atmeex_mask |= cMask;
  osal_pwrmgr_device(PWRMGR_ALWAYS_ON); // PWRMGR_ALWAYS_ON PWRMGR_BATTERY
  uart_wake_up_interrupt_disable();    // enable interrupt for RISING EDGE on RX pin of UART
}







#if defined(MY_SERIAL_APP_SERIAL_0)
static void ATMEEX_send_data_over_air0( void )
{
 /* 0x70 0x65 PANID PANID  N      DATA       LRC   
  *  [0]  [1]  [2]   [3]  [4]  [5, 6.. 3+N] [N+4]  
  *
  * N = BUFFER_LEN - 6   -->   BUFFER_LEN = N + 6
 */
  
  if (MySerialApp_Tx0Len == 0) return; // if no data - return

  uint8 buffer_len;

  if (MySerialApp_Tx0Buf[0] == 0x70) buffer_len = MySerialApp_Tx0Buf[4] + 6; // calculate sending buffer lenght
  else buffer_len = MySerialApp_Tx0Len; // calculate sending buffer lenght

  SendData( &zclSampleSw_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf );

  MySerialApp_Tx0Len = 0;
  
}
#endif


















#if defined(MY_SERIAL_APP_SERIAL_1)
static void ATMEEX_send_data_over_air1( void )
{  
  if ( MySerialApp_Tx1Len )
  {

    SendData( &zclSampleLight_DstAddr, COMMAND_SEND, MySerialApp_Tx1Len, MySerialApp_Tx1Buf );

  
#if MY_SERIAL_APP_ECHO
    HalUARTWrite(MY_SERIAL_APP_PORT_1, MySerialApp_Tx1Buf, MySerialApp_Tx1Len);
#endif
    
#if defined(MY_SERIAL_APP_SERIAL_0)
    HalUARTWrite(MY_SERIAL_APP_PORT_0, MySerialApp_Tx1Buf, MySerialApp_Tx1Len);
#endif
    //halSleepWait(50000);
    MySerialApp_Tx1Len = 0;
  }
}
#endif




/*
static void ATMEEX_BindNotificationCB( bdbBindNotificationData_t *data )
{
  //NLME_SetPollRate( 1000 ); // ATMEEX: for first start set poll rate 1 sec
  //osal_start_timerEx(zclSampleSw_TaskID, ATMEEX_SET_DEFAULT_POLL_RATE_ON_START, 15000); // set default poll rate
}
*/



void SendData( afAddrType_t *DstAddrPointer, uint8 cmd, uint8 dataBufLen, uint8 *dataBufPointer )
{
  //if (dataBufLen > 25) dataBufLen = 25; // protect for over buffering

  //uint8 *msgBuf;
  
  //msgBuf = osal_mem_alloc( dataBufLen );
 // if (msgBuf != NULL)
  //{
    zclSampleSwSeqNum++;
    zcl_SendCommand(SAMPLESW_ENDPOINT, DstAddrPointer, ZCL_CLUSTER_ID_GEN_ON_OFF, cmd, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, 0, zclSampleSwSeqNum, dataBufLen, dataBufPointer);
  //  osal_mem_free( msgBuf );
  //}
}

/*********************************************************************

 * @fn      MySerialApp_CalcLrc
 *
 * @brief   Calculating LRC
 *
 * @param   data - input buffer
 *          dataSize - size of input buffer
 *
 * @return  none
 */
static uint8 MySerialApp_CalcLrc(uint8 *data, size_t dataSize)
{
    if (data == NULL || dataSize == 0)
    {
        return 0;
    }

    uint8 lrc = 0;
    for (size_t n = 0; n < dataSize; ++n)
    {
        lrc ^= data[n];
    }
    return lrc;
}