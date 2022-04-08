/**************************************************************************************************
  Filename:       zcl_samplelight.h
  Revised:        $Date: 2014-06-19 08:38:22 -0700 (Thu, 19 Jun 2014) $
  Revision:       $Revision: 39101 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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

#ifndef ZCL_SAMPLELIGHT_H
#define ZCL_SAMPLELIGHT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "hal_types.h"

// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "zcl_general.h"
  #include "bdb_tlCommissioning.h"
#endif
/*********************************************************************
 * CONSTANTS
 */
#define SAMPLELIGHT_ENDPOINT            8

#define SAMPLELIGHT_NUM_GRPS            2 // Needed to include ZLL Target functionality
  
#define LIGHT_OFF                             0x00
#define LIGHT_ON                              0x01
#define REMOTE_HEADER                         0x02
// Application Events
//#define SAMPLELIGHT_POLL_CONTROL_TIMEOUT_EVT  0x0008
//#define SAMPLELIGHT_LEVEL_CTRL_EVT            0x0010
//#define SAMPLEAPP_END_DEVICE_REJOIN_EVT       0x0004

// UI Events
//#define SAMPLEAPP_LCD_AUTO_UPDATE_EVT         0x0010  
//#define SAMPLEAPP_KEY_AUTO_REPEAT_EVT         0x0020  

#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY     10000

#define ATMEEX_UART_DATA_READY_SERIAL0        0x0001
#define ATMEEX_UART_DATA_READY_SERIAL1        0x0002
#define SERIALAPP_CHECK_CONNECTION_EVT        0x0004
#define SERIALAPP_CHECK_CONN_RESET_EVT        0x0008 
#define SERIALAPP_SEND_REPLY_EVT              0x0010
#define SEND_RESPONSE                         0x0020
#define SECOND_SEND_UART_EVT                  0x0040
//#define DEBUG_RESET_NV_EVT                  0x0020

#define SERIALAPP_CHECK_CONNECTION_DELAY      5000
#define SERIALAPP_CHECK_CONN_RESET_DELAY      10000
#define SERIALAPP_SEND_REPLY_DELAY            15000 //10000

// delays from moment when we got get command for send response message 
#define ZC_SEND_RESPONSE_DELAY                3000 
#define ZRTR_SEND_RESPONSE_DELAY              5000
#define ZRTR_HUMIDIFIER_SEND_RESPONSE_DELAY   7500
  
// NVM IDs
#define NV_SERIALAPP_LAST_PANID               0x0402
#define NV_SERIALAPP_PANID                    0x0403
#define NV_SERIALAPP_PAIRING_STATE            0x0404
#define NV_SERIALAPP_RESTART_COUNTER          0x0405  
#define NV_SHORT_ADDRESS                      0x0406
//#define NV_PAIR_STATE_WAS_GOT                 0x0407

// Commands
#define COMMAND_SEND                          0x01
#define COMMAND_GET                           0x02
#define COMMAND_LOG                           0x00
//#define COMMAND_FROM_ED                       0x01 // from end device
//#define COMMAND_FROM_RTR                      0x02 // to coordinator

#define RESEND_TO_UART_TIMES_MAX 10
#define RESEND_TO_UART_PERIOD    1000

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
  
// Added to include ZLL Target functionality 
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  extern bdbTLDeviceInfo_t tlSampleLight_DeviceInfo;
#endif

  
extern SimpleDescriptionFormat_t zclSampleLight_SimpleDesc;

extern CONST zclCommandRec_t zclSampleLight_Cmds[];

extern CONST uint8 zclCmdsArraySize;

// attribute list
extern CONST zclAttrRec_t zclSampleLight_Attrs[];
extern CONST uint8 zclSampleLight_NumAttributes;

// Identify attributes
extern uint16 zclSampleLight_IdentifyTime;
extern uint8  zclSampleLight_IdentifyCommissionState;

// Groups attributes
extern uint8 zclSampleLight_GroupsNameSupport;

// Scenes attributes
extern uint8        zclSampleLight_ScenesSceneCount;
extern uint8        zclSampleLight_ScenesCurrentScene;
extern uint16       zclSampleLight_ScenesCurrentGroup;
extern uint8        zclSampleLight_ScenesSceneValid;
extern CONST uint8  zclSampleLight_ScenesNameSupport;

// OnOff attributes
extern uint8  zclSampleLight_OnOff;

// Level Control Attributes
#ifdef ZCL_LEVEL_CTRL
extern uint8  zclSampleLight_LevelCurrentLevel;
extern uint16 zclSampleLight_LevelRemainingTime;
extern uint16 zclSampleLight_LevelOnOffTransitionTime;
extern uint8  zclSampleLight_LevelOnLevel;
extern uint16 zclSampleLight_LevelOnTransitionTime;
extern uint16 zclSampleLight_LevelOffTransitionTime;
extern uint8  zclSampleLight_LevelDefaultMoveRate;
#endif

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleLight_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleLight_event_loop( byte task_id, UINT16 events );

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSampleLight_ResetAttributesToDefaultValues(void); //implemented in zcl_samplelight_data.c

/*********************************************************************
*********************************************************************/








#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLELIGHT_H */
