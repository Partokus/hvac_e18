/**************************************************************************************************
  Filename:       zcl_sampleLight.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $


  Description:    Zigbee Cluster Library - sample light application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This application implements a ZigBee Light, based on Z-Stack 3.0. It can be configured as an
  On/Off light or as a dimmable light, by undefining or defining ZCL_LEVEL_CTRL, respectively.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample applicetion.
  
  Application-specific UI peripherals being used:

  - LEDs:
    LED1 reflect the current light state (On / Off accordingly).

  Application-specific menu system:

    <TOGGLE LIGHT> Toggle the local light and display its status and level
      Press OK to toggle the local light on and off.
      This screen shows the following information
        Line1: (only populated if ZCL_LEVEL_CTRL is defined)
          LEVEL XXX - xxx is the current level of the light if the light state is ON, or the target level
            of the light when the light state is off. The target level is the level that the light will be
            set to when it is switched from off to on using the on or the toggle commands.
        Line2:
          LIGHT OFF / ON: shows the current state of the light.
      Note when ZCL_LEVEL_CTRL is enabled:
        - If the light state is ON and the light level is X, and then the light receives the OFF or TOGGLE 
          commands: The level will decrease gradually until it reaches 1, and only then the light state will
          be changed to OFF. The level then will be restored to X, with the state staying OFF. At this stage
          the light is not lighting, and the level represent the target level for the next ON or TOGGLE 
          commands.
        - If the light state is OFF and the light level is X, and then the light receives the ON or TOGGLE
          commands; The level will be set to 1, the light state will be set to ON, and then the level will
          increase gradually until it reaches level X.
        - Any level-setting command will affect the level directly, and may also affect the on/off state,
          depending on the command's arguments.       

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"
#include "AssocList.h"
#include "nwk_util.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_diagnostic.h"

#include "zcl_samplelight.h"
   
#include "bdb.h"
#include "bdb_interface.h"

 //GP_UPDATE
#include "gp_interface.h"
#include "hal_sleep.h"
#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "NLMEDE.h"

// Added to include TouchLink initiator functionality 
#if defined ( BDB_TL_INITIATOR )
  #include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR

#if defined ( BDB_TL_TARGET )
  #include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET

#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "bdb_touchlink.h"
#endif

//#include "zcl_sampleapps_ui.h"
      

/*
Settings

For HVAC:
HAL_UART_ISR=2
HAL_UART_DMA=0

for AIRNANNY:
HAL_UART_ISR=2
HAL_UART_DMA=1
UART0 - ESP32
UART1 - AVR

*/
      
/*********************************************************************
 * MACROS
 */
//#define UI_STATE_TOGGLE_LIGHT 1 //UI_STATE_BACK_FROM_APP_MENU is item #0, so app item numbers should start from 1

#define APP_TITLE "TI Sample Light"





#if (HAL_UART_DMA == 1) || (HAL_UART_ISR == 1)
#define MY_SERIAL_APP_SERIAL_0
#endif
#if (HAL_UART_DMA == 2) || (HAL_UART_ISR == 2)
#define MY_SERIAL_APP_SERIAL_1
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
#define MY_SERIAL_APP_RX_SZ  128
#endif

#if !defined( MY_SERIAL_APP_TX_SZ )
#define MY_SERIAL_APP_TX_SZ  128
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
#define MY_SERIAL_APP_TX_MAX  80
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
  ENABLE_JOINING_IN_NWK     = 6, // for Coordinator ?and router?. Enable joining in network for 180 sec
  PERMIT_JOINING_IN_NWK     = 7  // permit joining in nwk
};

#define NUM_OF_ELEMENTS(x) ( sizeof(x) / sizeof(x[0]) )

/* ATMEEX variables */
const uint8 headerSize = NUM_OF_ELEMENTS( headerArr );
devStates_t zclSampleLight_NwkState;
//bool ATMEEX_UART_AVAIL = 1;


/* ATMEEX functions */
static void resetFactorySettings( void );
static void enableJoiningInNwk( void );
static void permitJoiningInNwk( void );
static bool checkHeader( uint8* pBuf, uint8 len );
static void queryNetworkStatus( uint8 port );
static void readPANID( uint8 port );
static void setPANID( uint8 port, uint16 PANID );
static bool processCommand( uint8* pBuf, uint8 len, uint8 port );
#if defined(MY_SERIAL_APP_SERIAL_1)
static void PortMapUart( uint8 PortAlt, uint8 PortNum );
#endif
static void atmeex_init( void );
static void atmeex_keys_init( void );
static bool led_stat = 0;
void atmeex_led_on( void );
void atmeex_led_off( void );
void atmeex_led_toggle( void );
void cleanupChildTableFxn( void );

uint8 isHaveChild = 0;

//static void ATMEEX_send_test_data( void );
#if defined(MY_SERIAL_APP_SERIAL_0)
static void ATMEEX_send_data_over_air0( void );
static void MySerialApp_CallBack0( uint8 port, uint8 event );
#endif

#if defined(MY_SERIAL_APP_SERIAL_1)
static void ATMEEX_send_data_over_air1( void );
static void MySerialApp_CallBack1( uint8 port, uint8 event );
#endif 
//static void ATMEEX_BindNotificationCB( bdbBindNotificationData_t *data );
//static uint8 atmeex_MsgID = 0;
//uint8 debug_reset = 0;
//void ATMEEX_proccess_pairing( void );

//static uint8 sendPairCmdAccept[] = {0x70, 0x65, 0xAB, 0xCD, 0x06, 'C', 'M', 'D', 0x01, 0x00, 0x05, 0x4F}; 
//void sendPairAccept( void );
uint8 address_saved = 0;
//uint16 atmeex_short_address = 0x0404;
uint8 SendData( afAddrType_t *DstAddrPointer, uint8 cmd, uint8 dataBufLen, uint8 *dataBufPointer );

uint8 response_buffer[25];
uint8 response_buffer_len = 0;

#if ZG_BUILD_COORDINATOR_TYPE && AIRNANNY
uint8 response_buffer_airnanny[25];
uint8 response_buffer_len_airnanny;
#endif


bool atmeex_pair_state_was_got = 0;





#if ATMEEX_HUMIDIFIER_ROUTER
uint8 resend_to_uart_times = 0; // repit send to uart 
//uint8 humidifier_error = 0;
uint8 second_uart_send_buffer[25];
static bool MySerialApp_PairingState = false;
static bool MySerialApp_PairingState_2 = 0;
static uint16 MySerialApp_PanId = 0;
static uint8 MySerialApp_RestartCounter = 0;
static uint8 sendPairingReq[] = {0x70, 0x65, 0x66, 0x57, 0x06, 'C', 'M', 'D', 0x00, 0x00, 0x05, 0x4F};
//static uint8 sendPairingSuccessefull[] = {0x70, 0x65, 0x66, 0x57, 0x06, 0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x05, 0x4F};
static const uint8 responceHeader[] = {'C', 'M', 'D'};
static const uint8 remoteHeader = 2;
static const uint8 humidifierHeader = 5;
static const uint8 cmdAccept = 0x01;
static const uint16 zigbeePreamble = 0x6570;
static const uint16 humidifierPreamble = 0xAA55;
static const uint16 sharedConnectionId = 0x6657;

typedef struct
{
  uint16 preamble;
  uint16 connectionId;
  uint8 *src;
  size_t srcSize;
  uint8 *dst;
  size_t dstSize;
  size_t outSize;
} packet_t;

typedef struct
{
  uint8 header;
  uint8 cmd;
  bool saveSettings;
  bool powerOn;
  bool autoOn;
  bool nightOn;
  uint8 userBlowerSpeed;
  uint8 userDamperPosition;
  uint8 userHumidity;
  float userTemperature;
  uint8 refWorkMode;
  uint8 refFanSpeed;
  float refTempSetp;
} remoteState_t;

typedef struct
{
  uint8 header;
  uint8 lastCmd;
  bool fillWater;
  bool powerOn;
  uint8 humidifierStage;
} humidifierState_t;

typedef struct
{
  uint16 preamble;
  uint8 dataSize;
  uint8 version;
  bool powerOn;
  uint8 mode;
  union 
  {
    uint8 modeAuto;
    uint8 setHumidity;
    uint8 manual;
  } modeValue;
  uint8 peripheralHeat:1;
  uint8 peripheralAnion:1;
  uint8 peripheralLight:1;
  uint8 peripheralSleep:1;
  uint16 time;
  bool fillWater;
  uint8 enviromentTemperature;
  uint8 enviromentHumidity;
  uint8 networkConfig;
  uint8 networkStatus;
  uint8 reserve[3];
  uint8 dataflowDirection;
  uint8 lrc;
} humidifierDataFormat_t;

//static uint8 debData[64];
static remoteState_t remoteState;
static humidifierState_t humidifierState;
static humidifierDataFormat_t humidifierData;


static bool MySerialApp_GetPacket(packet_t *packet);
static bool MySerialApp_ParceHumidData( uint8 *pBuf, size_t len );
static uint8 MySerialApp_CalcLrc(uint8 *data, size_t dataSize);

//static bool secret_light_on = 0; // secret light. 0 - off, 1 - on.
#endif /* ATMEEX_HUMIDIFIER_ROUTER */








/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleLight_TaskID;
uint8 zclSampleLightSeqNum = 0;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleLight_DstAddr;
#if (ZG_BUILD_COORDINATOR_TYPE)
afAddrType_t zclSampleLight_Breezer_DstAddr;
afAddrType_t zclSampleLight_Humidifier_DstAddr;
#else
afAddrType_t zclSampleLight_Coordinator_DstAddr;
#endif
afAddrType_t zclSampleLight_Broadcast_DstAddr;
afAddrType_t zclSampleLight_Groupcast_DstAddr;


// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleLight_TestEp =
{
  SAMPLELIGHT_ENDPOINT,
  0,
  &zclSampleLight_TaskID,
  (SimpleDescriptionFormat_t *)&zclSampleLight_SimpleDesc,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

#ifdef ZCL_LEVEL_CTRL
uint8 zclSampleLight_WithOnOff;       // set to TRUE if state machine should set light on/off
uint8 zclSampleLight_NewLevel;        // new level when done moving
uint8 zclSampleLight_LevelChangeCmd; // current level change was triggered by an on/off command
bool  zclSampleLight_NewLevelUp;      // is direction to new level up or down?
int32 zclSampleLight_CurrentLevel32;  // current level, fixed point (e.g. 192.456)
int32 zclSampleLight_Rate32;          // rate in units, fixed point (e.g. 16.123)
uint8 zclSampleLight_LevelLastLevel;  // to save the Current Level before the light was turned OFF
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void zclSampleLight_HandleKeys( byte shift, byte keys );
//static void zclSampleLight_BasicResetCB( void );
static void zclSampleLight_OnOffCB( uint8 cmd );

//static void zclSampleLight_ReportCB( uint8 cmd );
//GP_UPDATE
#if (ZG_BUILD_RTR_TYPE)
//static void gp_CommissioningMode(bool isEntering);
//static uint8 gp_ChangeChannelReq(void);
#endif


static void zclSampleLight_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);


#ifdef ZCL_LEVEL_CTRL
static void zclSampleLight_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd );
static void zclSampleLight_LevelControlMoveCB( zclLCMove_t *pCmd );
static void zclSampleLight_LevelControlStepCB( zclLCStep_t *pCmd );
static void zclSampleLight_LevelControlStopCB( void );
static void zclSampleLight_DefaultMove( uint8 OnOff );
static uint32 zclSampleLight_TimeRateHelper( uint8 newLevel );
static uint16 zclSampleLight_GetTime ( uint8 level, uint16 time );
static void zclSampleLight_MoveBasedOnRate( uint8 newLevel, uint32 rate );
static void zclSampleLight_MoveBasedOnTime( uint8 newLevel, uint16 time );
static void zclSampleLight_AdjustLightLevel( void );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleLight_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

//static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);

//void zclSampleLight_UiActionToggleLight(uint16 keys);
//void zclSampleLight_UiUpdateLcd(uint8 uiCurrentState, char * line[3]);
//void zclSampleLight_UpdateLedState(void);



#define LEVEL_CHANGED_BY_LEVEL_CMD  0
#define LEVEL_CHANGED_BY_ON_CMD     1
#define LEVEL_CHANGED_BY_OFF_CMD    2

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
  const char sLightOn[]      = "   LIGHT ON     ";
  const char sLightOff[]     = "   LIGHT OFF    ";
  #ifdef ZCL_LEVEL_CTRL
    char sLightLevel[]        = "   LEVEL ###    "; // displays level 1-254
  #endif
#endif

/*********************************************************************
 * REFERENCED EXTERNALS
 */
extern int16 zdpExternalStateTaskID;

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleLight_CmdCallbacks =
{
  NULL, //zclSampleLight_BasicResetCB,            // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  zclSampleLight_OnOffCB,                 // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  zclSampleLight_LevelControlMoveToLevelCB, // Level Control Move to Level command
  zclSampleLight_LevelControlMoveCB,        // Level Control Move command
  zclSampleLight_LevelControlStepCB,        // Level Control Step command
  zclSampleLight_LevelControlStopCB,        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleLight_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleLight_Init( byte task_id )
{
  zclSampleLight_TaskID = task_id;

  
  // Groupcast's address
  zclSampleLight_Groupcast_DstAddr.addrMode = (afAddrMode_t)AddrGroup;
  zclSampleLight_Groupcast_DstAddr.endPoint = 8;
  zclSampleLight_Groupcast_DstAddr.addr.shortAddr = 0x000A;
  
  // Broadcast's address
  zclSampleLight_Broadcast_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_Broadcast_DstAddr.endPoint = 8;
  zclSampleLight_Broadcast_DstAddr.addr.shortAddr = 0xFFFF;
  
#if (ZG_BUILD_COORDINATOR_TYPE)
  // Set destination address to indirect

  // Remote control's address
  zclSampleLight_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_DstAddr.endPoint = 8;
  zclSampleLight_DstAddr.addr.shortAddr = 0x0404;
  
  
  // Breezer's address
  zclSampleLight_Breezer_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_Breezer_DstAddr.endPoint = 8;
  zclSampleLight_Breezer_DstAddr.addr.shortAddr = 0x0404;
  
  // Humidifier's address
  zclSampleLight_Humidifier_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_Humidifier_DstAddr.endPoint = 8;
  zclSampleLight_Humidifier_DstAddr.addr.shortAddr = 0x0404;
  
#else
  
  // Set destination address to indirect
  zclSampleLight_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_DstAddr.endPoint = 8;
  zclSampleLight_DstAddr.addr.shortAddr = 0x0404;
  
  zclSampleLight_Coordinator_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclSampleLight_Coordinator_DstAddr.endPoint = 8;
  zclSampleLight_Coordinator_DstAddr.addr.shortAddr = 0x0000;
  
#endif
  
  atmeex_init();
  
  
  
  
  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &zclSampleLight_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_ENDPOINT, &zclSampleLight_CmdCallbacks );

  // Register the application's attribute list
  zclSampleLight_ResetAttributesToDefaultValues();
  zcl_registerAttrList( SAMPLELIGHT_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );

#ifdef ZCL_LEVEL_CTRL
  zclSampleLight_LevelLastLevel = zclSampleLight_LevelCurrentLevel;
#endif

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleLight_TaskID );

#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SAMPLELIGHT_ENDPOINT, zclCmdsArraySize, zclSampleLight_Cmds );
#endif

  // Register low voltage NV memory protection application callback
  //RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleLight_TaskID );
  //bdb_RegisterBindNotificationCB(ATMEEX_BindNotificationCB);
  bdb_RegisterCommissioningStatusCB( zclSampleLight_ProcessCommissioningStatus );
  
  // Register for a test endpoint
  afRegister( &sampleLight_TestEp );

#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SAMPLELIGHT_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif
  
//GP_UPDATE  
#if (ZG_BUILD_RTR_TYPE)  
  //gp_RegisterCommissioningModeCB(gp_CommissioningMode);
  //gp_RegisterGPChangeChannelReqCB(gp_ChangeChannelReq);
#endif
  
  zdpExternalStateTaskID = zclSampleLight_TaskID;

#if !AIRNANNY
  cleanupChildTableFxn();  
#endif
  
  
#if (ZG_BUILD_COORDINATOR_TYPE)
 
#if AIRNANNY
  bdb_StartCommissioning(/*BDB_COMMISSIONING_MODE_NWK_STEERING |*/ BDB_COMMISSIONING_MODE_NWK_FORMATION /* | BDB_COMMISSIONING_MODE_FINDING_BINDING */);
#else
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_NWK_FORMATION /* | BDB_COMMISSIONING_MODE_FINDING_BINDING */);
#endif

#else
  

  if ( SUCCESS == osal_nv_item_init( NV_SERIALAPP_LAST_PANID, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId ) )
  {
    osal_nv_read( NV_SERIALAPP_LAST_PANID, 0, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId );
  }
  

  
#if ATMEEX_HUMIDIFIER_ROUTER 
  if ( SUCCESS == osal_nv_item_init( NV_SERIALAPP_PANID, sizeof(MySerialApp_PanId), &MySerialApp_PanId ) )
  {
    osal_nv_read( NV_SERIALAPP_PANID, 0, sizeof(MySerialApp_PanId), &MySerialApp_PanId );
  }
  
  if ( SUCCESS == osal_nv_item_init( NV_SERIALAPP_PAIRING_STATE, sizeof(MySerialApp_PairingState), &MySerialApp_PairingState ) )
  {
    osal_nv_read( NV_SERIALAPP_PAIRING_STATE, 0, sizeof(MySerialApp_PairingState), &MySerialApp_PairingState );
  }
  
  if ( SUCCESS == osal_nv_item_init( NV_SERIALAPP_RESTART_COUNTER, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter ) )
  {
    osal_nv_read( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
  }
 
  
  ++MySerialApp_RestartCounter;
  osal_nv_write( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
  osal_start_timerEx( zclSampleLight_TaskID, SERIALAPP_CHECK_CONN_RESET_EVT, SERIALAPP_CHECK_CONN_RESET_DELAY );
  
  //remoteState.header = 2;
  humidifierState.header = humidifierHeader;
  humidifierState.lastCmd = 0;
  humidifierState.fillWater = false;
  humidifierState.powerOn = false;
  humidifierState.humidifierStage = 0;
  osal_start_reload_timer( zclSampleLight_TaskID, SERIALAPP_SEND_REPLY_EVT, SERIALAPP_SEND_REPLY_DELAY );
#endif /* ATMEEX_HUMIDIFIER_ROUTER */  
  
  
  
  
  

  
  
    //atmeex_led_on();
  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING/* | BDB_COMMISSIONING_MODE_FINDING_BINDING */);
    
  
 
  //osal_start_timerEx(zclSampleLight_TaskID, DEBUG_RESET_NV_EVT, 10000);
#endif
  
  if ( SUCCESS == osal_nv_item_init( NV_SHORT_ADDRESS, sizeof(zclSampleLight_DstAddr.addr.shortAddr), &zclSampleLight_DstAddr.addr.shortAddr ) )
  {
    osal_nv_read( NV_SHORT_ADDRESS, 0, sizeof(zclSampleLight_DstAddr.addr.shortAddr), &zclSampleLight_DstAddr.addr.shortAddr );
    //zclSampleLight_DstAddr.addr.shortAddr = atmeex_short_address;
  }
  
  
   
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
uint16 zclSampleLight_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter


  
  if ( events & SYS_EVENT_MSG )
  {
    
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleLight_TaskID )) )
    {
      
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
           
          zclSampleLight_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;
        case AF_INCOMING_MSG_CMD:
          //atmeex_led_off();
          break;
        case KEY_CHANGE:
          //zclSampleLight_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleLight_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (zclSampleLight_NwkState == DEV_ZB_COORD) ||
          (zclSampleLight_NwkState == DEV_ROUTER) ||
          (zclSampleLight_NwkState == DEV_END_DEVICE) )
          {
            
          #if (ZG_BUILD_COORDINATOR_TYPE)
          //atmeex_led_on();
          #else
          //NLME_PermitJoiningRequest(0); // permit child make this device own parent
          //atmeex_led_on();
          #endif
          
          }
          
          break;
          
        case AF_DATA_CONFIRM_CMD:
          #if (ZG_BUILD_COORDINATOR_TYPE)
          //atmeex_led_toggle();  
          #else
          //atmeex_led_toggle();
          #endif
          
          break;
          
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
    //debug_reset = 1;
    if ( processCommand( MySerialApp_Tx0Buf, MySerialApp_Tx0Len, MY_SERIAL_APP_PORT_0 ) )
    {
      MySerialApp_Tx0Len = 0;
    }
#if ATMEEX_HUMIDIFIER_ROUTER
    else if( MySerialApp_ParceHumidData( MySerialApp_Tx0Buf, MySerialApp_Tx0Len ) )
    {
      MySerialApp_Tx0Len = 0;
    }
#endif /* ATMEEX_HUMIDIFIER_ROUTER */
    else
    {
#if !ATMEEX_HUMIDIFIER_ROUTER
      
     ATMEEX_send_data_over_air0();
     
#else 
     
     MySerialApp_Tx0Len = 0;
     
#endif
    }

    //ATMEEX_POWER_SAVING_MODE_TURN_ON(); // turn on power saving mode
    // return unprocessed events
    return ( events ^ ATMEEX_UART_DATA_READY_SERIAL0 );
  }
#endif
  
  
  
  
  
  
  
  
#if defined(MY_SERIAL_APP_SERIAL_1)
  
  if ( events & ATMEEX_UART_DATA_READY_SERIAL1 )
  {

    
    if ( processCommand( MySerialApp_Tx1Buf, MySerialApp_Tx1Len, MY_SERIAL_APP_PORT_1 ) )
    {
      MySerialApp_Tx1Len = 0;
      
    }
#if ATMEEX_HUMIDIFIER_ROUTER
    else if( MySerialApp_ParceHumidData( MySerialApp_Tx1Buf, MySerialApp_Tx1Len ) )
    {
      MySerialApp_Tx1Len = 0;
    }
#endif /* ATMEEX_HUMIDIFIER_ROUTER */
    else
    {

      ATMEEX_send_data_over_air1();
    }
    // return unprocessed events
    return ( events ^ ATMEEX_UART_DATA_READY_SERIAL1 );
  }
#endif
  
  
  
  
  

if ( events & SEND_RESPONSE )
  {

#if !AIRNANNY
    if (response_buffer_len != 0)
    {
#if (ZG_BUILD_COORDINATOR_TYPE)
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, response_buffer_len, response_buffer ); // send ref state to remote pult
#else  
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, response_buffer_len, response_buffer ); // send ref state to remote pult
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_LOG, response_buffer_len, response_buffer ); // send log to coordinator
#endif /* ZG_BUILD_COORDINATOR_TYPE */ 
    
      response_buffer_len = 0;
    } 
#if !VERTICAL_HVAC 
    //else // send error code. 0xAA - not data came from UART
    //{ 
    //  uint8 buf_t[1] = {0xB0};
    //  SendData( &zclSampleLight_DstAddr, COMMAND_SEND, 1, buf_t );
    //}
#endif
#endif
    
    
#if AIRNANNY
    if (response_buffer_len_airnanny != 0)
    {
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, response_buffer_len_airnanny, response_buffer_airnanny ); // send ref state to remote pult

      response_buffer_len_airnanny = 0;
    } //else // send error code. 0xAA - not data came from UART
   // { 
    //  uint8 buf_t[1] = {0xB0};
    //  SendData( &zclSampleLight_DstAddr, COMMAND_SEND, 1, buf_t );
    //}
#endif
    
    return ( events ^ SEND_RESPONSE );
  }
  
 
  
  
  
  
  
  
  
#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SAMPLEAPP_END_DEVICE_REJOIN_EVT );
  }
#endif
  
  
  
  
  

  
  
  
  
#if ATMEEX_HUMIDIFIER_ROUTER
  
  if ( events & SECOND_SEND_UART_EVT )
  {
    HalUARTWrite( MY_SERIAL_APP_PORT_0, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
    MicroWait(50000);
    MicroWait(50000);
    HalUARTWrite( MY_SERIAL_APP_PORT_0, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
    
    resend_to_uart_times--;  
    if (resend_to_uart_times != 0) osal_start_timerEx( zclSampleLight_TaskID, SECOND_SEND_UART_EVT, RESEND_TO_UART_PERIOD );
    
    return ( events ^ SECOND_SEND_UART_EVT );
  }
  
  
  if ( events & SERIALAPP_CHECK_CONNECTION_EVT )
  {
    if(MySerialApp_PairingState)
    {
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_SEND, sizeof(sendPairingReq), sendPairingReq );
      MySerialApp_RestartCounter = 0;
      MySerialApp_PairingState = false;
      MySerialApp_PairingState_2 = true;
      osal_nv_write( NV_SERIALAPP_PAIRING_STATE, 0, sizeof(MySerialApp_PairingState), &MySerialApp_PairingState );
      osal_nv_write( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
    } 
    
    return ( events ^ SERIALAPP_CHECK_CONNECTION_EVT );
  }

  
  
  
  

  
  
  if ( events & SERIALAPP_CHECK_CONN_RESET_EVT )
  {
    
    if(MySerialApp_RestartCounter >= 3)
    {
      if (!MySerialApp_PairingState)
      {
      MySerialApp_PairingState = true;
      osal_nv_write( NV_SERIALAPP_PAIRING_STATE, 0, sizeof(MySerialApp_PairingState), &MySerialApp_PairingState );
      MySerialApp_PanId = 0;
      osal_nv_write( NV_SERIALAPP_PANID, 0, sizeof(MySerialApp_PanId), &MySerialApp_PanId );

      resetFactorySettings(); 
      }
    } else {
      MySerialApp_RestartCounter = 0;
      osal_nv_write( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
    }
    
    return ( events ^ SERIALAPP_CHECK_CONN_RESET_EVT );
  }
  
  
  
  
  
  
  if ( events & SERIALAPP_SEND_REPLY_EVT )
  {
    uint8 lrc = MySerialApp_CalcLrc((uint8 *)&humidifierState, sizeof(humidifierState));
    uint8 reply[] = { (uint8)zigbeePreamble, (uint8)(zigbeePreamble >> 8), (uint8)MySerialApp_PanId, (uint8)(MySerialApp_PanId >> 8),
                      sizeof(humidifierState), humidifierState.header, humidifierState.lastCmd, humidifierState.fillWater, 
                      humidifierState.powerOn, humidifierState.humidifierStage, lrc };


    if (isHaveChild) // if this node has child
    {
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_LOG, sizeof(reply), reply ); // send log to coordinator
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, sizeof(reply), reply );
    } else {
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, sizeof(reply), reply );
    }
    
    

    return ( events ^ SERIALAPP_SEND_REPLY_EVT );
  }
#endif /* ATMEEX_HUMIDIFIER_ROUTER */
  
  
  
  
  
  
  
  
  /*
  if ( events & DEBUG_RESET_NV_EVT )
  {

    SendData( &zclSampleLight_DstAddr, COMMAND_SEND, 25, atmeex_message );

    osal_start_timerEx(zclSampleLight_TaskID, DEBUG_RESET_NV_EVT, 4000);
    
    return ( events ^ DEBUG_RESET_NV_EVT );
  }
  */
  
  
  
  
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      zclSampleLight_HandleKeys
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
static void zclSampleLight_HandleKeys( byte shift, byte keys )
{
  //UI_MainStateMachine(keys);
}
*/

//GP_UPDATE
#if (ZG_BUILD_RTR_TYPE)
/*********************************************************************
 * @fn      gp_CommissioningMode
 *
 * @brief   Callback that notifies the application that gp Proxy is entering 
 *          into commissioning mode
 *
 * @param   isEntering - 
 *
 * @return  
 */
/*
static void gp_CommissioningMode(bool isEntering)
{
  if(isEntering)
  {
    //Led on indicating enter commissioning mode
  }
  else
  {
    //Led off indicating enter commissioning mode
  }
}
*/


//GP_UPDATE
/*********************************************************************
 * @fn      gp_ChangeChannelReq
 *
 * @brief   Callback function to notify the application about a GP commissioning 
 * request that will change the current channel for at most 
 * gpBirectionalCommissioningChangeChannelTimeout ms
 *
 * @param   channel - Channel in which the commissioning will take place
 *
 * @return  TRUE to allow change channel, FALSE to do not allow
 */
/*
static uint8 gp_ChangeChannelReq(void)
{
  bool allowChangeChannel = TRUE;
  
  //Check application state to decide if allow change channel or not
  
  return allowChangeChannel;
}
*/
#endif


/*********************************************************************
 * @fn      zclSampleLight_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleLight_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
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
          MySerialApp_LastPanId = _NIB.nwkPanId;
          osal_nv_write( NV_SERIALAPP_LAST_PANID, 0, sizeof(MySerialApp_LastPanId), &MySerialApp_LastPanId );
          //Forbid_or_allow_power_saving_mode(1); // allow power saving mode
        }

#if ATMEEX_HUMIDIFIER_ROUTER         
         if(MySerialApp_PairingState && MySerialApp_RestartCounter >= 4)
         {
           // MySerialApp_RestartCounter = 0;
           // osal_nv_write( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
            osal_start_timerEx( zclSampleLight_TaskID, SERIALAPP_CHECK_CONNECTION_EVT, SERIALAPP_CHECK_CONNECTION_DELAY );
         }
#endif /* ATMEEX_HUMIDIFIER_ROUTER */
#endif /* ZG_BUILD_JOINING_TYPE */
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
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        osal_start_timerEx(zclSampleLight_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);
      }
    break;
#endif 
  }
  
  //UI_UpdateComissioningStatus(bdbCommissioningModeMsg);
}

/*********************************************************************
 * @fn      zclSampleLight_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
/*
static void zclSampleLight_BasicResetCB( void )
{
  //Reset every attribute in all supported cluster to their default value.

  zclSampleLight_ResetAttributesToDefaultValues();

  zclSampleLight_UpdateLedState();

  // update the display
  //UI_UpdateLcd( ); 
}
*/

























#define BREEZER_HEADER 0x01
#define REMOTE_HEADER 0x02
#define CONDITIONEER_HEADER 0x04
#define HUMIDIFIER_HEADER 0x05

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
  
  /*
  uint8 cleanupChildTable = TRUE;
//zgSetItem (ZCD_NV_ROUTER_OFF_ASSOC_CLEANUP, sizeof (cleanupChildTable),
//& cleanupChildTable);

  osal_nv_read( ZCD_NV_ROUTER_OFF_ASSOC_CLEANUP, 0, sizeof(cleanupChildTable), &cleanupChildTable );
  uint8 buf_t[2];
  buf_t[0] = 0xAA;
  buf_t[1] = cleanupChildTable;

SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, 2, buf_t );
*/
  
 
  
  if (cmd != COMMAND_LOG && pPtr->srcAddr.addr.shortAddr != 0x0000) // if it is not LOG and if it is not coordinator, so it is Remote Control and save short address 
  { 
      zclSampleLight_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
      zclSampleLight_DstAddr.endPoint = 8;
     
    if (address_saved == 0) // save address of Remote control in first message 
    {
      osal_nv_write( NV_SHORT_ADDRESS, 0, sizeof(pPtr->srcAddr.addr.shortAddr), &(pPtr->srcAddr.addr.shortAddr) );
      address_saved = 1; // never come into this "if" untill reset
    }
  }
  
  
  
  
  switch (cmd)
  {
  case COMMAND_GET: // if we got get request, so send response

#if !AIRNANNY
  isHaveChild = AssocIsChild(pPtr->srcAddr.addr.shortAddr); // if it is his child - isHaveChild = 1
#endif

#if (ZG_BUILD_COORDINATOR_TYPE)
    //osal_start_timerEx( zclSampleLight_TaskID, SEND_RESPONSE, ZC_SEND_RESPONSE_DELAY );
#else  
#if ATMEEX_HUMIDIFIER_ROUTER
    //osal_start_timerEx( zclSampleLight_TaskID, SEND_RESPONSE, ZRTR_HUMIDIFIER_SEND_RESPONSE_DELAY);
#else
    //osal_start_timerEx( zclSampleLight_TaskID, SEND_RESPONSE, ZRTR_SEND_RESPONSE_DELAY );
#endif /* ATMEEX_HUMIDIFIER_ROUTER */
#endif /* ZG_BUILD_COORDINATOR_TYPE */ 
    
    // Release the memory
    //osal_msg_deallocate( (uint8 *)pPtr );
    return;
    
    
    
  case COMMAND_SEND:   
    break;
    
    
    
 
 
  case COMMAND_LOG: // if it LOG from routers - just send through UART
    // if this node has child, so relay message to ZED
    if (isHaveChild) SendData( &zclSampleLight_DstAddr, COMMAND_SEND, (pPtr->cmd.DataLength - 3), &(pPtr->cmd.Data[3]) ); 

#if !ZG_BUILD_COORDINATOR_TYPE   
    
    if (pPtr->cmd.Data[3] == 0x75 && pPtr->cmd.Data[8] != REMOTE_HEADER) return; // if it is not OTA and if it is not REMOTE HEADER, just exit and don't send to uart
     
#endif /* ZG_BUILD_COORDINATOR_TYPE */ 

    /*
    if (pPtr->cmd.Data[8] == BREEZER_HEADER)  // look at header of packet (6-th byte) and save address for unicast messaging
    {
      zclSampleLight_Breezer_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
      zclSampleLight_Breezer_DstAddr.endPoint = 8;
    } else if (pPtr->cmd.Data[8] == HUMIDIFIER_HEADER)
    {
      zclSampleLight_Humidifier_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
      zclSampleLight_Humidifier_DstAddr.endPoint = 8;
    }     
   */   
    break;
               

    
  }


  
  
  
  
  
  
  
 
#if ATMEEX_HUMIDIFIER_ROUTER 
       
    packet_t packet;
    uint8 dataBuffer[64];
    
    //if (MySerialApp_PairingState_2)
    if (0)
    {
      packet.preamble = zigbeePreamble;
      packet.connectionId = sharedConnectionId;
      packet.src = &(pPtr->cmd.Data[3]);
      packet.srcSize = pPtr->cmd.DataLength - 3;
      packet.dst = dataBuffer;
      packet.dstSize = sizeof(dataBuffer);
      
      if(MySerialApp_GetPacket(&packet))
      {
        for(int i = 0; i < sizeof(responceHeader); ++i)
        {
          if(packet.dst[i] != responceHeader[i])
          {
            // Release the memory
            //osal_msg_deallocate( (uint8 *)pPtr );
            return;
          }
        }
        
        if(packet.dst[sizeof(responceHeader)] == cmdAccept)
        {    
        MySerialApp_PanId = *(uint16*)&packet.dst[sizeof(responceHeader) + sizeof(cmdAccept)];
        osal_nv_write( NV_SERIALAPP_PANID, 0, sizeof(MySerialApp_PanId), &MySerialApp_PanId );
        MySerialApp_PairingState = false;      
        osal_nv_write( NV_SERIALAPP_PAIRING_STATE, 0, sizeof(MySerialApp_PairingState), &MySerialApp_PairingState );
        MySerialApp_RestartCounter = 0;
        osal_nv_write( NV_SERIALAPP_RESTART_COUNTER, 0, sizeof(MySerialApp_RestartCounter), &MySerialApp_RestartCounter );
        
        osal_start_reload_timer( zclSampleLight_TaskID, SERIALAPP_SEND_REPLY_EVT, SERIALAPP_SEND_REPLY_DELAY );
        
        atmeex_pair_state_was_got = 1;
        }
        
      }
    }
    else
    {
      packet.preamble = zigbeePreamble;
      packet.connectionId = MySerialApp_PanId;
      packet.src = &(pPtr->cmd.Data[3]);
      packet.srcSize = pPtr->cmd.DataLength - 3;
      packet.dst = (uint8 *)&remoteState;
      packet.dstSize = sizeof(remoteState);
      
      if(MySerialApp_GetPacket(&packet))
      {
        if(remoteState.header == remoteHeader)
        {
          humidifierData.preamble = humidifierPreamble;
          humidifierData.dataSize = sizeof(humidifierDataFormat_t);
          humidifierData.version = 1;
          humidifierData.powerOn = remoteState.userHumidity > 0 ? remoteState.powerOn : false;
          humidifierData.mode = 2; // manual
          humidifierData.modeValue.manual = remoteState.userHumidity;
          humidifierData.peripheralHeat = 1;          //on
          humidifierData.peripheralAnion = 1;         //on
          humidifierData.peripheralLight = 0;         //off
          humidifierData.peripheralSleep = 1;         //on
          humidifierData.time = 0;
          humidifierData.networkConfig = 0; // TO DO
          humidifierData.networkStatus = 0; // TO DO
          humidifierData.dataflowDirection = 0;
          humidifierData.lrc = MySerialApp_CalcLrc((uint8 *)&humidifierData, (sizeof(humidifierDataFormat_t) - 1));
          
          humidifierState.lastCmd = remoteState.cmd;
       
          humidifierState.humidifierStage = remoteState.userHumidity;

#if defined(MY_SERIAL_APP_SERIAL_0)
          HalUARTWrite( MY_SERIAL_APP_PORT_0, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
          MicroWait(50000);
          MicroWait(50000);
          HalUARTWrite( MY_SERIAL_APP_PORT_0, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
          
          resend_to_uart_times = RESEND_TO_UART_TIMES_MAX;
          
          osal_start_timerEx( zclSampleLight_TaskID, SECOND_SEND_UART_EVT, RESEND_TO_UART_PERIOD );        
#endif
          
#if defined(MY_SERIAL_APP_SERIAL_1)

          //HalUARTWrite( MY_SERIAL_APP_PORT_1, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
          //MicroWait(100000);
          //HalUARTWrite( MY_SERIAL_APP_PORT_1, (uint8 *)&humidifierData, sizeof(humidifierDataFormat_t) );
          
#endif
          //osal_set_event( MySerialApp_TaskID, SERIALAPP_SEND_REPLY_EVT );
        } //else humidifier_error = 4;
      } //else humidifier_error = 5;
    }

  
    
    /*
   uint8 buf_t[2];
   buf_t[0] = 0xaa;
   buf_t[1] = humidifier_error;

   SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_LOG, 2, buf_t ); // send log to coordinator
    */
    
#else    
    
    
    
    

#if defined(MY_SERIAL_APP_SERIAL_0)
    HalUARTWrite( MY_SERIAL_APP_PORT_0, &(pPtr->cmd.Data[3]), (pPtr->cmd.DataLength - 3) ); // 0,1,2 byte - not TMEEX's application data
#endif
    
#if defined(MY_SERIAL_APP_SERIAL_1)
    HalUARTWrite( MY_SERIAL_APP_PORT_1, &(pPtr->cmd.Data[3]), (pPtr->cmd.DataLength - 3) ); // 0,1,2 byte - not ATMEEX's application data    
#endif 
      
    
    
    
#endif /* ATMEEX_HUMIDIFIER_ROUTER  */   

    
    
    
    
    
    
#if ZG_BUILD_JOINING_TYPE && !ATMEEX_HUMIDIFIER_ROUTER
    /*
    if (MySerialApp_Tx0Buf[5] == 'C' && MySerialApp_Tx0Buf[6] == 'M' && MySerialApp_Tx0Buf[7] == 'D' && MySerialApp_Tx0Buf[8] == 0x01) // if got pair state from coordinator
    {
      atmeex_pair_state_was_got = 1; 
    }
    */
#endif        
         
    
    
    
    
    
    
    // Release the memory
    //osal_msg_deallocate( (uint8 *)pPtr );
}






























#ifdef ZCL_LEVEL_CTRL
/*********************************************************************
 * @fn      zclSampleLight_TimeRateHelper
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 *
 * @return  diff (directly), zclSampleLight_CurrentLevel32 and zclSampleLight_NewLevel, zclSampleLight_NewLevelUp
 */
static uint32 zclSampleLight_TimeRateHelper( uint8 newLevel )
{
  uint32 diff;
  uint32 newLevel32;

  // remember current and new level
  zclSampleLight_NewLevel = newLevel;
  zclSampleLight_CurrentLevel32 = (uint32)1000 * zclSampleLight_LevelCurrentLevel;

  // calculate diff
  newLevel32 = (uint32)1000 * newLevel;
  if ( zclSampleLight_LevelCurrentLevel > newLevel )
  {
    diff = zclSampleLight_CurrentLevel32 - newLevel32;
    zclSampleLight_NewLevelUp = FALSE;  // moving down
  }
  else
  {
    diff = newLevel32 - zclSampleLight_CurrentLevel32;
    zclSampleLight_NewLevelUp = TRUE;   // moving up
  }

  return ( diff );
}

/*********************************************************************
 * @fn      zclSampleLight_MoveBasedOnRate
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 * @param   rate16   - fixed point rate (e.g. 16.123)
 *
 * @return  none
 */
static void zclSampleLight_MoveBasedOnRate( uint8 newLevel, uint32 rate )
{
  uint32 diff;

  // determine how much time (in 10ths of seconds) based on the difference and rate
  zclSampleLight_Rate32 = rate;
  diff = zclSampleLight_TimeRateHelper( newLevel );
  zclSampleLight_LevelRemainingTime = diff / rate;
  if ( !zclSampleLight_LevelRemainingTime )
  {
    zclSampleLight_LevelRemainingTime = 1;
  }

  osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      zclSampleLight_MoveBasedOnTime
 *
 * @brief   Calculate rate based on time, and startup level state machine
 *
 * @param   newLevel  - new level for current level
 * @param   time      - in 10ths of seconds
 *
 * @return  none
 */ 
static void zclSampleLight_MoveBasedOnTime( uint8 newLevel, uint16 time )
{
  uint16 diff;

  // determine rate (in units) based on difference and time
  diff = zclSampleLight_TimeRateHelper( newLevel );
  zclSampleLight_LevelRemainingTime = zclSampleLight_GetTime( newLevel, time );
  zclSampleLight_Rate32 = diff / time;

  osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      zclSampleLight_GetTime
 *
 * @brief   Determine amount of time that MoveXXX will take to complete.
 *
 * @param   level = new level to move to
 *          time  = 0xffff=default, or 0x0000-n amount of time in tenths of seconds.
 *
 * @return  none
 */
static uint16 zclSampleLight_GetTime( uint8 newLevel, uint16 time )
{
  
  // there is a hiearchy of the amount of time to use for transistioning
  // check each one in turn. If none of defaults are set, then use fastest
  // time possible.
  if ( time == 0xFFFF )
  {
    // use On or Off Transition Time if set (not 0xffff)
    if ( zclSampleLight_LevelCurrentLevel > newLevel )
    {
      time = zclSampleLight_LevelOffTransitionTime;
    }
    else
    {
      time = zclSampleLight_LevelOnTransitionTime;
    }

    // else use OnOffTransitionTime if set (not 0xffff)
    if ( time == 0xFFFF )
    {
      time = zclSampleLight_LevelOnOffTransitionTime;
    }

    // else as fast as possible
    if ( time == 0xFFFF )
    {
      time = 1;
    }
  }

  if ( time == 0 )
  {
    time = 1; // as fast as possible
  }

  return ( time );

}

/*********************************************************************
 * @fn      zclSampleLight_DefaultMove
 *
 * @brief   We were turned on/off. Use default time to move to on or off.
 *
 * @param   zclSampleLight_OnOff - must be set prior to calling this function.
 *
 * @return  none
 */
static void zclSampleLight_DefaultMove( uint8 OnOff )
{
  
  uint8  newLevel;
  uint32 rate;      // fixed point decimal (3 places, eg. 16.345)
  uint16 time;

  // if moving to on position, move to on level
  if ( OnOff )
  {
    if (zclSampleLight_OnOff == LIGHT_OFF)
    {
      zclSampleLight_LevelCurrentLevel = ATTR_LEVEL_MIN_LEVEL;
    }
    
    if ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT )
    {
      // The last Level (before going OFF) should be used)
      newLevel = zclSampleLight_LevelLastLevel;
    }
    else
    {
      newLevel = zclSampleLight_LevelOnLevel;
    }

    time = zclSampleLight_LevelOnTransitionTime;

  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL;

    time = zclSampleLight_LevelOffTransitionTime;
  }

  // else use OnOffTransitionTime if set (not 0xffff)
  if ( time == 0xFFFF )
  {
    time = zclSampleLight_LevelOnOffTransitionTime;
  }

  // else as fast as possible
  if ( time == 0xFFFF )
  {
    time = 1;
  }

  // calculate rate based on time (int 10ths) for full transition (1-254)
  rate = 255000 / time;    // units per tick, fixed point, 3 decimal places (e.g. 8500 = 8.5 units per tick)

  // start up state machine.
  zclSampleLight_WithOnOff = TRUE;
  zclSampleLight_MoveBasedOnRate( newLevel, rate );

}

/*********************************************************************
 * @fn      zclSampleLight_AdjustLightLevel
 *
 * @brief   Called each 10th of a second while state machine running
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_AdjustLightLevel( void )
{
  
  // one tick (10th of a second) less
  if ( zclSampleLight_LevelRemainingTime )
  {
    --zclSampleLight_LevelRemainingTime;
  }

  // no time left, done
  if ( zclSampleLight_LevelRemainingTime == 0)
  {
    zclSampleLight_LevelCurrentLevel = zclSampleLight_NewLevel;
  }

  // still time left, keep increment/decrementing
  else
  {
    if ( zclSampleLight_NewLevelUp )
    {
      zclSampleLight_CurrentLevel32 += zclSampleLight_Rate32;
    }
    else
    {
      zclSampleLight_CurrentLevel32 -= zclSampleLight_Rate32;
    }
    zclSampleLight_LevelCurrentLevel = (uint8)( zclSampleLight_CurrentLevel32 / 1000 );
  }

  if (( zclSampleLight_LevelChangeCmd == LEVEL_CHANGED_BY_LEVEL_CMD ) && ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT ))
  {
    zclSampleLight_LevelLastLevel = zclSampleLight_LevelCurrentLevel;
  }

  // also affect on/off
  if ( zclSampleLight_WithOnOff )
  {
    if ( zclSampleLight_LevelCurrentLevel > ATTR_LEVEL_MIN_LEVEL )
    {
      zclSampleLight_OnOff = LIGHT_ON;
    }
    else
    {
      if (zclSampleLight_LevelChangeCmd != LEVEL_CHANGED_BY_ON_CMD)
      {
        zclSampleLight_OnOff = LIGHT_OFF;
      }
      else
      {
        zclSampleLight_OnOff = LIGHT_ON;
      }
      
      if (( zclSampleLight_LevelChangeCmd != LEVEL_CHANGED_BY_LEVEL_CMD ) && ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT ))
      {
        zclSampleLight_LevelCurrentLevel = zclSampleLight_LevelLastLevel;
      }
    }
  }

  zclSampleLight_UpdateLedState();
  
  // display light level as we go
  //UI_UpdateLcd( );

  // keep ticking away
  if ( zclSampleLight_LevelRemainingTime )
  {
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
  }

}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlMoveToLevelCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMoveToLevel Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd )
{
  
  zclSampleLight_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  zclSampleLight_WithOnOff = pCmd->withOnOff;
  zclSampleLight_MoveBasedOnTime( pCmd->level, pCmd->transitionTime );

}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlMoveCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMove Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlMoveCB( zclLCMove_t *pCmd )
{
  
  uint8 newLevel;
  uint32 rate;

  // convert rate from units per second to units per tick (10ths of seconds)
  // and move at that right up or down
  zclSampleLight_WithOnOff = pCmd->withOnOff;

  if ( pCmd->moveMode == LEVEL_MOVE_UP )
  {
    newLevel = ATTR_LEVEL_MAX_LEVEL;  // fully on
  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL; // fully off
  }

  zclSampleLight_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  rate = (uint32)100 * pCmd->rate;
  zclSampleLight_MoveBasedOnRate( newLevel, rate );

}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlStepCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlStepCB( zclLCStep_t *pCmd )
{
  
  uint8 newLevel;

  // determine new level, but don't exceed boundaries
  if ( pCmd->stepMode == LEVEL_MOVE_UP )
  {
    if ( (uint16)zclSampleLight_LevelCurrentLevel + pCmd->amount > ATTR_LEVEL_MAX_LEVEL )
    {
      newLevel = ATTR_LEVEL_MAX_LEVEL;
    }
    else
    {
      newLevel = zclSampleLight_LevelCurrentLevel + pCmd->amount;
    }
  }
  else
  {
    if ( pCmd->amount >= zclSampleLight_LevelCurrentLevel )
    {
      newLevel = ATTR_LEVEL_MIN_LEVEL;
    }
    else
    {
      newLevel = zclSampleLight_LevelCurrentLevel - pCmd->amount;
    }
  }
  
  zclSampleLight_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  // move to the new level
  zclSampleLight_WithOnOff = pCmd->withOnOff;
  zclSampleLight_MoveBasedOnTime( newLevel, pCmd->transitionTime );

}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlStopCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Level Control Stop Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlStopCB( void )
{
  // stop immediately
  osal_stop_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT );
  zclSampleLight_LevelRemainingTime = 0;
}
#endif

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
 * @fn      zclSampleLight_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleLight_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleLight_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleLight_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_CONFIG_REPORT:
    case ZCL_CMD_CONFIG_REPORT_RSP:
    case ZCL_CMD_READ_REPORT_CFG:
    case ZCL_CMD_READ_REPORT_CFG_RSP:
    case ZCL_CMD_REPORT:
      
      //bdb_ProcessIncomingReportingMsg( pInMsg );
      break;

    case ZCL_CMD_DEFAULT_RSP:
      zclSampleLight_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleLight_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleLight_ProcessInDiscAttrsExtRspCmd( pInMsg );
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
 * @fn      zclSampleLight_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
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

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleLight_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER
/*
void zclSampleLight_UiActionToggleLight(uint16 keys)
{
  zclSampleLight_OnOffCB(COMMAND_TOGGLE);
}
*/
/*
void zclSampleLight_UpdateLedState(void)
{
  // set the LED1 based on light (on or off)
  if ( zclSampleLight_OnOff == LIGHT_ON )
  {
    //HalLedSet ( UI_LED_APP, HAL_LED_MODE_ON );
  }
  else
  {
    //HalLedSet ( UI_LED_APP, HAL_LED_MODE_OFF );
  }
}
*/
/*
void zclSampleLight_UiUpdateLcd(uint8 UiState, char * line[3])
{
#ifdef LCD_SUPPORTED
#ifdef ZCL_LEVEL_CTRL
  zclHA_uint8toa( zclSampleLight_LevelCurrentLevel, &sLightLevel[9] );
  line[0] = (char *)sLightLevel;
#endif // ZCL_LEVEL_CTRL
  line[1] = (char *)(zclSampleLight_OnOff ? sLightOn : sLightOff);
  line[2] = "< TOGGLE LIGHT >";
#endif
}
*/
/****************************************************************************
****************************************************************************/







static void atmeex_keys_init( void )
{
   #if !ZG_BUILD_JOINING_TYPE 
   P1SEL &= ~BV(1); // gpio func
   P1DIR |= BV(1); // output
   P1_1 = 0;
   #else
#if AIRNANNY
   P1SEL &= ~BV(0); // gpio func
   P1DIR |= BV(0); // output
   P1_0 = 0; 
#else
   P0SEL &= ~BV(6); // gpio func
   P0DIR |= BV(6); // output
   P0_6 = 0; 
#endif
   #endif
}



void atmeex_led_on( void )
{
  #if !ZG_BUILD_JOINING_TYPE 
   P1_1 = 1;
   #else
#if AIRNANNY
   P1_0 = 1;
#else
   P0_6 = 1;  
#endif
        
   #endif
}

void atmeex_led_off( void )
{
  #if !ZG_BUILD_JOINING_TYPE 
   P1_1 = 0;
   #else
   #if AIRNANNY
   P1_0 = 0;
#else
   P0_6 = 0;  
#endif      
   #endif
}

void atmeex_led_toggle( void )
{
  led_stat = !led_stat;
  
  #if !ZG_BUILD_JOINING_TYPE 
  P1_1 = led_stat;
  #else
  #if AIRNANNY
   P1_0 = led_stat;
#else
   P0_6 = led_stat;  
#endif
      
  #endif
}



static void atmeex_init( void )
{
  atmeex_keys_init();

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
  
  
  
  // add yourself in group
  aps_Group_t group;
  // Assign yourself to group A
  group.ID = 0x000A;
  group.name[0] = 6; // First byte is string length
  osal_memcpy( &(group.name[1]), "GroupA", 6);
  aps_AddGroup( SAMPLELIGHT_ENDPOINT, &group );
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
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
      (MySerialApp_Tx0Len < MY_SERIAL_APP_TX_MAX))
  {
    MySerialApp_Tx0Len += HalUARTRead(MY_SERIAL_APP_PORT_0, MySerialApp_Tx0Buf + MySerialApp_Tx0Len,
                                      MY_SERIAL_APP_TX_MAX - MySerialApp_Tx0Len);
    
    osal_start_timerEx( zclSampleLight_TaskID, ATMEEX_UART_DATA_READY_SERIAL0, 50 );
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
    
    osal_start_timerEx( zclSampleLight_TaskID, ATMEEX_UART_DATA_READY_SERIAL1, 50 );
  }
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
   // if (len > headerSize + 2)
  //  {
      setPANID(port, pBuf[headerSize + 1] << 8 | pBuf[headerSize + 2]);
  //  }
    break;
    
  case READ_PANID:
    readPANID(port);
    break;
  
  case ENABLE_JOINING_IN_NWK:
    enableJoiningInNwk();
    break;
    
  case PERMIT_JOINING_IN_NWK:
    permitJoiningInNwk();
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



static void queryNetworkStatus( uint8 port )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0 };
  answer[5] = devState;
  HalUARTWrite( port, answer, NUM_OF_ELEMENTS( answer ) );
}

static void readPANID( uint8 port )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  uint16 PANID = _NIB.nwkPanId;
  answer[5] = PANID >> 8;
  answer[6] = PANID;

  HalUARTWrite(port, answer, NUM_OF_ELEMENTS(answer));
}


static void setPANID( uint8 port, uint16 PANID )
{
  uint8 answer[] = { 0, 0, 0, 0, 0, 0 };
  
  resetFactorySettings();
  
  // bdb_resetLocalAction(); 

  answer[4] = 0x4F;
  answer[5] = 0x4B;
  
  HalUARTWrite(port, answer, NUM_OF_ELEMENTS(answer));
}


// enable joining in nwk for 180 sec
static void enableJoiningInNwk( void )
{
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING); 
}

// disable joining in nwk
static void permitJoiningInNwk( void )
{
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_IDDLE); 
}





static void resetFactorySettings( void )
{
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


  zgWriteStartupOptions( ZG_STARTUP_SET, (ZCD_STARTOPT_DEFAULT_NETWORK_STATE | ZCD_STARTOPT_DEFAULT_CONFIG_STATE) );
  SystemReset();
}


/*
static void ATMEEX_send_test_data( void )
{
  //uint8 atmeex_message[25] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};     
  //zcl_SendCommand(SAMPLELIGHT_ENDPOINT, &zclSampleLight_DstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF, COMMAND_TOGGLE, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, FALSE, 0, zcl_seqNum, 25, atmeex_message);
  char theMessageData[2] = {0xAA, 0xBB};
  atmeex_MsgID++;
  if ( AF_DataRequest( &zclSampleLight_DstAddr, &sampleLight_TestEp,
                       ATMEEX_CLUSTER_ID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &atmeex_MsgID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    atmeex_led_on();
  } else {
    atmeex_led_off();
  }
}
*/









//uint8 error_code1 = 0xf9;

/* AIRNANNY ESP32 */
#if defined(MY_SERIAL_APP_SERIAL_0) && !ATMEEX_HUMIDIFIER_ROUTER
static void ATMEEX_send_data_over_air0( void )
{
  if ( MySerialApp_Tx0Len )
  {
    
    //if (MySerialApp_Tx0Buf[0] == 0x70 || MySerialApp_Tx0Buf[0] == 0x02) // if first byte correct 
    //{
    uint8 buffer_len;
    
    
    if (MySerialApp_Tx0Buf[0] == 0x70) buffer_len = MySerialApp_Tx0Buf[4] + 6; // calculate sending buffer lenght
    else buffer_len = MySerialApp_Tx0Len; // calculate sending buffer lenght
    
#if !(ATMEEX_HUMIDIFIER_ROUTER) && !(ZG_BUILD_COORDINATOR_TYPE) // if not humidifier

    /*
    if ((MySerialApp_Tx0Buf[5] == 'C' && MySerialApp_Tx0Buf[6] == 'M' && MySerialApp_Tx0Buf[7] == 'D') || (MySerialApp_Tx0Buf[0] == 0x02 && MySerialApp_Tx0Buf[1] == 0xA8))
    {
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf );
    } else 
    {
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf );
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf );
    }
    */
    
    if (isHaveChild) // if this node has child
    {
      
      SendData( &zclSampleLight_Coordinator_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf );
      if (MySerialApp_Tx0Buf[0] != 0x02) SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf ); // if it is not OTA packet, send message to remote control  

    } else {
      
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf );
     
    }
    
    
    
    
    
    //status_t = SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf );
      
    //if (status_t != 0 && MySerialApp_Tx0Buf[0] != 0x02) SendData( &zclSampleLight_Broadcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf ); // if first sended message was unsuccessful 
                                                                                                                                                      // and it is not OTA header, send just broadcast
    //else SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf );                                                  // otherwise send groupcast
                                               
    /* good for OTA
    status_t = SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx0Buf ); 
    if (status_t != 0 && error_code1 != status_t) error_code1 = status_t;
    */
    
    //NWK_INDIRECT_MSG_MAX_PER  12

#endif /* !(ATMEEX_HUMIDIFIER_ROUTER) && !(ZG_BUILD_COORDINATOR_TYPE)*/
    
    
    
    
    
    
    
    
#if ZG_BUILD_COORDINATOR_TYPE && AIRNANNY // if AIRNANNY


    /*
    if ((MySerialApp_Tx0Buf[5] == 'C' && MySerialApp_Tx0Buf[6] == 'M' && MySerialApp_Tx0Buf[7] == 'D') || (MySerialApp_Tx0Buf[0] == 0x02 && MySerialApp_Tx0Buf[1] == 0xA8))
    {
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf );
    } else 
    {
      osal_memcpy(response_buffer_airnanny, MySerialApp_Tx0Buf, buffer_len);
      response_buffer_len_airnanny = buffer_len;
    }
    */
    
    SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx0Buf ); 

#endif 
   
    
    
    
    

  
#if MY_SERIAL_APP_ECHO
    HalUARTWrite(MY_SERIAL_APP_PORT_0, MySerialApp_Tx0Buf, buffer_len);
#endif
    
#if defined(MY_SERIAL_APP_SERIAL_1)
    HalUARTWrite(MY_SERIAL_APP_PORT_1, MySerialApp_Tx0Buf, buffer_len);
#endif
    
    
  //}
  MySerialApp_Tx0Len = 0;
  }
  
}

#endif













//uint8 status_error = 0;
//uint8 status_error_counter = 0;

/* AIRNANNY AVR */
#if defined(MY_SERIAL_APP_SERIAL_1)
static void ATMEEX_send_data_over_air1( void )
{  

  if ( MySerialApp_Tx1Len ) // if we have some data
  {
    
    //if (MySerialApp_Tx1Buf[0] == 0x70 || MySerialApp_Tx1Buf[0] == 0x02) // if first byte correct 
    //{
    uint8 buffer_len;
    
    
    
    if (MySerialApp_Tx1Buf[0] == 0x70) buffer_len = MySerialApp_Tx1Buf[4] + 6; // calculate sending buffer lenght
    else buffer_len = MySerialApp_Tx1Len; // calculate sending buffer lenght

#if !AIRNANNY
    
    
    
  /*
    if ((MySerialApp_Tx1Buf[5] == 'C' && MySerialApp_Tx1Buf[6] == 'M' && MySerialApp_Tx1Buf[7] == 'D' && MySerialApp_Tx1Buf[8] == 0x01) || (MySerialApp_Tx1Buf[0] == 0x02 && MySerialApp_Tx1Buf[1] == 0xA8))
    {
      SendData( &zclSampleLight_Broadcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf ); // Broadcast
    } else 
    {
      osal_memcpy(response_buffer, MySerialApp_Tx1Buf, buffer_len);
      response_buffer_len = buffer_len;
    }
  */
    
#if ZG_BUILD_COORDINATOR_TYPE
    /*
     if (MySerialApp_Tx1Buf[5] == 0x02 || MySerialApp_Tx1Buf[0] == 0x02) // if header equal Remote header (if it command from Mobile App)
     {
       SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
       SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
     } else
     {
       SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
       //if (status_t != 0) cleanupChildTableFxn(); // if not success, so clear child table
     }   
   */
    
    //uint8 status_t;
    
    if (isHaveChild) // if this node has child
    {
      
      if (MySerialApp_Tx1Buf[5] == 0x02 || MySerialApp_Tx1Buf[0] == 0x02) // if header equal Remote header (if it command from Mobile App) or it is OTA header 
      {
        SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx1Buf ); // send groupcast
      }
      
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );

    } else {
      
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_LOG, buffer_len, MySerialApp_Tx1Buf );
     
    }
    
    /*
    if (MySerialApp_Tx1Buf[5] == 0x02 || MySerialApp_Tx1Buf[0] == 0x02) // if header equal Remote header (if it command from Mobile App) or it is OTA header 
    {
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf ); // send groupcast
    }
    SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
    */
    
    
    
    //status_t = SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
    /*
    if (status_t != 0) {
      status_error = status_t;
      status_error_counter = 5;
    }
    else {
      status_error_counter--;
      if (status_error_counter > 5) status_error_counter = 0;
    }
    
    if (status_error_counter != 0) {
      uint8 buf_t[2] = {0xAA, 0x00};
      buf_t[1] = status_error;
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_SEND, 2, buf_t ); 
    }
*/
    
    
    
    
    /*
      uint8 status_t;
      
      status_t = SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
      
      if (status_t != 0) SendData( &zclSampleLight_Broadcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf ); // if first sended message was unsuccessful send just broadcast
      else {
        if (MySerialApp_Tx1Buf[5] == 0x02 || MySerialApp_Tx1Buf[0] == 0x02) // if header equal Remote header (if it command from Mobile App) or it is OTA header 
        {
          SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );               // otherwise send groupcast
        }
      }
      */
      /* good for OTA
      SendData( &zclSampleLight_Groupcast_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf ); 
      */
      
#endif    /*ZG_BUILD_COORDINATOR_TYPE*/
   
#else // if AIRNANNY


    SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf ); 
    /*
    if ((MySerialApp_Tx1Buf[5] == 'C' && MySerialApp_Tx1Buf[6] == 'M' && MySerialApp_Tx1Buf[7] == 'D') || (MySerialApp_Tx1Buf[0] == 0x02 && MySerialApp_Tx1Buf[1] == 0xA8))
    {
      SendData( &zclSampleLight_DstAddr, COMMAND_SEND, buffer_len, MySerialApp_Tx1Buf );
    } else 
    {
      osal_memcpy(response_buffer, MySerialApp_Tx1Buf, buffer_len);
      response_buffer_len = buffer_len;
    }
    */ 
 
#endif    


    
    
    
    
    
    
    
    //HalUARTWrite(MY_SERIAL_APP_PORT_1, MySerialApp_Tx1Buf, buffer_len);
#if MY_SERIAL_APP_ECHO
    HalUARTWrite(MY_SERIAL_APP_PORT_1, MySerialApp_Tx1Buf, buffer_len);
#endif
    
#if defined(MY_SERIAL_APP_SERIAL_0)
    HalUARTWrite(MY_SERIAL_APP_PORT_0, MySerialApp_Tx1Buf, buffer_len);
#endif
    
    
  //}
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

















#if ZG_BUILD_JOINING_TYPE
#if ATMEEX_HUMIDIFIER_ROUTER

/*********************************************************************

 * @fn      MySerialApp_GetPacket
 *
 * @brief   Check and receive Zigbee packet
 *
 * @param   packet - pointer to meta data and received data
 *
 * @return  false - if packet check failed
 *          true - if packet check passed
 */
static bool MySerialApp_GetPacket(packet_t *packet)
{
  uint8 *src = (uint8 *)packet->src;
  uint8 *dst = (uint8 *)packet->dst;
  uint8 packetDataSize = 0;
  uint8 lrc = 0;
  packet->outSize = 0;
  
  do
  {
    if (packet->preamble != *(uint16*)&src[packet->outSize])
    {
      //humidifier_error = 1;
      break;
    }
    packet->outSize += sizeof(packet->preamble);
    
    if (packet->connectionId != *(uint16*)&src[packet->outSize])
    {
    //  break;
    }
    packet->outSize += sizeof(packet->connectionId);
    
    packetDataSize = src[packet->outSize];
    packet->outSize += sizeof(packetDataSize);
    
    if(packetDataSize > packet->dstSize)
    {
      //humidifier_error = 2;
      break;
    }
    
    lrc = MySerialApp_CalcLrc(&src[packet->outSize], packetDataSize);
    
    for (int i = packet->outSize; i < (packet->outSize + packetDataSize); ++i)
    {
      dst[i - packet->outSize] = src[i];
    }
    packet->outSize += packetDataSize;
    
    if(lrc != src[packet->outSize])
    {
      //humidifier_error = 3;
      break;
    }
    return true;
  }
  while(0);
  return false;
}

/*********************************************************************

 * @fn      MySerialApp_ParceHumidData
 *
 * @brief   Check and parce humidifier data
 *
 * @param   packet - pointer to meta data and received data
 *
 * @return  false - if packet check failed
 *          true - if packet check passed
 */
static bool MySerialApp_ParceHumidData( uint8 *pBuf, size_t len )
{
  humidifierDataFormat_t *humidifierData = (humidifierDataFormat_t *)pBuf;
  do
  {
    if (humidifierPreamble != humidifierData->preamble)
    {
      break;
    }
    
    if (sizeof(humidifierDataFormat_t) != humidifierData->dataSize)
    {
      break;
    }
    
    if(MySerialApp_CalcLrc(pBuf, len - 1) != humidifierData->lrc)
    {
      break;
    }
    
    humidifierState.header = humidifierHeader;
    humidifierState.fillWater = humidifierData->fillWater;
    humidifierState.powerOn = humidifierData->powerOn;
    humidifierState.humidifierStage = humidifierData->modeValue.manual;
    humidifierState.lastCmd++;
    osal_stop_timerEx( zclSampleLight_TaskID, SECOND_SEND_UART_EVT );   
    return true;
  }
  while(0);
  return false;
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


#endif /*ATMEEX_HUMIDIFIER_ROUTER*/
#endif /*ZG_BUILD_JOINING_TYPE*/

/*
#if !(ZG_BUILD_JOINING_TYPE)
void sendPairAccept( void )
{
  uint8 *msgBuf;
  msgBuf = osal_mem_alloc( sizeof(sendPairCmdAccept) );
  if (msgBuf != NULL)
  {

    sendPairCmdAccept[11] = MySerialApp_CalcLrc(sendPairCmdAccept, 0x06); // get LRC
             
    zcl_SendCommand(SAMPLELIGHT_ENDPOINT, &zclSampleLight_Broadcast_DstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF, COMMAND_PAIR, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, 0, 0, sizeof(sendPairCmdAccept), sendPairCmdAccept);
    osal_mem_free( msgBuf );
  }
}

#endif
*/



uint8 SendData( afAddrType_t *DstAddrPointer, uint8 cmd, uint8 dataBufLen, uint8 *dataBufPointer )
{
  //if (dataBufLen > 25) dataBufLen = 25; // protect for over buffering
  
 // uint8 *msgBuf;
  uint8 status_t;
 // msgBuf = osal_mem_alloc( dataBufLen );
 // if (msgBuf != NULL)
 // {
    zclSampleLightSeqNum++;
    
    status_t = zcl_SendCommand(SAMPLELIGHT_ENDPOINT, DstAddrPointer, ZCL_CLUSTER_ID_GEN_ON_OFF, cmd, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, 0, zclSampleLightSeqNum, dataBufLen, dataBufPointer);

  //  osal_mem_free( msgBuf );
    
    return status_t;
  //}
  //return 0;
}



void cleanupChildTableFxn( void )
{
  uint8 cleanupChildTable = TRUE;
zgSetItem (ZCD_NV_ROUTER_OFF_ASSOC_CLEANUP, sizeof (cleanupChildTable),
& cleanupChildTable);
}
















