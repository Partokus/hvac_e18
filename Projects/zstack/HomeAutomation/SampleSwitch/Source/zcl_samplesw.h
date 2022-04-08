/**************************************************************************************************
  Filename:       zcl_samplesw.h
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $


  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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

#ifndef ZCL_SAMPLESW_H
#define ZCL_SAMPLESW_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "hal_types.h"
/*********************************************************************
 * CONSTANTS
 */
#define SAMPLESW_ENDPOINT               8

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Events for the sample app
#define SAMPLEAPP_END_DEVICE_REJOIN_EVT                0x01
#define ATMEEX_UART_DATA_READY_SERIAL0                 0x02
#define ATMEEX_UART_DATA_READY_SERIAL1                 0x04
#define ATMEEX_POWER_SAVING_MODE_ON                    0x08
#define ATMEEX_SET_DEFAULT_POLL_RATE_ON_START          0x10
#define SAMPLESW_TOGGLE_TEST_EVT                       0x20
#define SEND_GET_REQUEST                               0x40
#define STOP_OTA_FOR_AVR_TIMER                         0x80

#define SEND_MESSAGE_BY_BUTTONS_DELAY                  400
#define SEND_MESSAGE_BY_MODE_BUTTON_DELAY              2500
  
#define OTA_STOP_DELAY                                 30000
#define DELAY_BEFORE_SLEEP                             40


  
// UI Events
//#define SAMPLEAPP_LCD_AUTO_UPDATE_EVT              0x0010  
//#define SAMPLEAPP_KEY_AUTO_REPEAT_EVT              0x0020 


#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY        10000
#define SAMPLEAPP_END_DEVICE_REJOIN_SHORT_DELAY  1000
#define SEND_GET_REQUEST_DEFAULT_PERIOD          4000  
#define POLL_RATE_FOR_MOBILE_APP_DELAY           10000
// Commands
#define COMMAND_SEND                            0x01
#define COMMAND_GET                             0x02

// For powermode
#define ONOFFCB_MASK                            0x01 // mask for permit sleep mode inside zclSampleLight_OnOffCB function
#define INTERRUPT_WAKE_UP_MASK                  0x02 // mask for permit sleep mode inside interrupt for wake up E18
#define COMMAND_FROM_AVR_MASK                   0x04
#define OTA_FOR_AVR_MASK                        0x08
#define atmeex_set_sleep_mask(x) atmeex_mask |= (x)
#define atmeex_clear_sleep_mask(x) atmeex_mask &= ~((x))


// Pressed buttons
#define ON_OFF_BUTTON 41
#define DAMPER_BUTTON 42 // (IN_EX)
#define PID_BUTTON 43
#define MODE_BUTTON 44
#define COND_AND_BREEZER_BUTTONS 45
#define HUMIDIFIER_BUTTONS 46
#define TEMP_BUTTONS 47
  
static uint8 short_or_default_delay_counter = 0; // if < 10 - short delay, if >= 10 - default delay. need for fast rejoin to parent for first time if connection have lost
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleSw_SimpleDesc;

extern SimpleDescriptionFormat_t zclSampleSw9_SimpleDesc;

extern CONST zclAttrRec_t zclSampleSw_Attrs[];

extern uint8  zclSampleSw_OnOff;

// OnOff attributes
extern bool zclSampleLight_OnOff;
extern uint16 tempData;
extern uint16 zclSampleSw_IdentifyTime;

extern uint8 zclSampleSw_OnOffSwitchType;

extern uint8 zclSampleSw_OnOffSwitchActions;

extern CONST uint8 zclSampleSw_NumAttributes;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleSw_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleSw_event_loop( byte task_id, UINT16 events );

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSampleSw_ResetAttributesToDefaultValues(void); //implemented in zcl_samplesw_data.c




/*********************************************************************
*********************************************************************/






#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEAPP_H */
