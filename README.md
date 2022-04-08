# Z-Stack 3.0.2
____
# Инструкция по сборке и компиляции проекта для вывода прошивки для:
1. A7   - Airnanny       - Coordinator
2. HVAC - Conditioner    - Coordinator
3. HVAC - Breezer        - Router
4. HVAC - Humidifier     - Router
5. HVAC - Remote control - End Device
____



Изменённые файлы SDK:
1. Z-Stack_3.0.2\Components\hal\target\CC2530EB\_hal_uart_isr.c
2. Z-Stack_3.0.2\Components\hal\target\CC2530EB\hal_sleep.c
3. Z-Stack_3.0.2\Components\mac\high_level\mac_pib.c
4. Z-Stack_3.0.2\Components\stack\nwk\nwk_globals.c
5. Z-Stack_3.0.2\Components\stack\sys\ZGlobals.c
6. Z-Stack_3.0.2\Projects\zstack\Tools\CC2530DB\f8wConfig.cfg
7. Z-Stack_3.0.2\Projects\zstack\Tools\CC2530DB\f8wEndev.cfg
8. Z-Stack_3.0.2\Projects\zstack\ZMain\TI2530DB\ZMain.c
____
1. В файле Z-Stack_3.0.2\Components\hal\target\CC2530EB\_hal_uart_isr.c в функции uint16 HalUARTReadISR(uint8 *buf, uint16 len)
Было:
uint16 cnt = 0;

Стало:
uint16 cnt = 0;

#ifdef UART_WAKE_UP
  if (isrCfg.rxBuf[isrCfg.rxHead] == 0xAA) 
  {
    isrCfg.rxHead++; 
    if (isrCfg.rxHead >= HAL_UART_ISR_RX_MAX)
    {
      isrCfg.rxHead = 0;
    }
    len--;
  }  
#endif
____
2. В файле Z-Stack_3.0.2\Components\hal\target\CC2530EB\hal_sleep.c в функции void halSleep( uint32 osal_timeout )
Было:
		/* Prep CC2530 power mode */
        HAL_SLEEP_PREP_POWER_MODE(halPwrMgtMode);

        /* save interrupt enable registers and disable all interrupts */
        HAL_SLEEP_IE_BACKUP_AND_DISABLE(ien0, ien1, ien2);
        HAL_ENABLE_INTERRUPTS();

Стало:
		/* Prep CC2530 power mode */
        HAL_SLEEP_PREP_POWER_MODE(halPwrMgtMode);

        /* save interrupt enable registers and disable all interrupts */
        HAL_SLEEP_IE_BACKUP_AND_DISABLE(ien0, ien1, ien2);
        HAL_ENABLE_INTERRUPTS();
        
        HalUARTSuspend(); // ATMEEX: UART turn off
        P0IFG = 0; P0IF = 0; P0IEN |= BV(2); // ATMEEX: uart_wake_up_interrupt_enable
____
3. В файле Z-Stack_3.0.2\Components\mac\high_level\mac_pib.c в static CODE const macPib_t macPibDefaults =
Было: 
  /* Proprietary */
  0,                                         /* phyTransmitPower */ 
  MAC_CHAN_11,                               /* logicalChannel */
  
Стало:
  /* Proprietary */
#if (HAL_PA_LNA)
  19,                                         /* phyTransmitPower */ 
#else
  3,                                          /* phyTransmitPower */ 
#endif                                              
  MAC_CHAN_11,                                /* logicalChannel */
____
4. В файле Z-Stack_3.0.2\Components\stack\nwk\nwk_globals.c
Было:
// Maximums for the data buffer queue
#define NWK_MAX_DATABUFS_WAITING    8 // Waiting to be sent to MAC
#define NWK_MAX_DATABUFS_SCHEDULED  5 // Timed messages to be sent
#define NWK_MAX_DATABUFS_CONFIRMED  5 // Held after MAC confirms
#define NWK_MAX_DATABUFS_TOTAL      12 // Total number of buffers

Стало:
// Maximums for the data buffer queue
#define NWK_MAX_DATABUFS_WAITING    16 // Waiting to be sent to MAC
#define NWK_MAX_DATABUFS_SCHEDULED  10 // Timed messages to be sent
#define NWK_MAX_DATABUFS_CONFIRMED  10 // Held after MAC confirms
#define NWK_MAX_DATABUFS_TOTAL      24 // Total number of buffers

Было:
#define NWK_INDIRECT_CNT_RTG_TMR    60

Стало:
#define NWK_INDIRECT_CNT_RTG_TMR    1
____
5. В файле Z-Stack_3.0.2\Components\stack\sys\ZGlobals.c
Было:
// devices upon joining.
uint8 zgPreConfigKeys = FALSE;

Стало:
// devices upon joining.
uint8 zgPreConfigKeys = TRUE;
____
6. В файле Z-Stack_3.0.2\Projects\zstack\Tools\CC2530DB\f8wConfig.cfg множество изменений - просто заменить файл.
____
7. В файле Z-Stack_3.0.2\Projects\zstack\Tools\CC2530DB\f8wEndev.cfg
Было:
-DMAC_CFG_TX_DATA_MAX=3
-DMAC_CFG_TX_MAX=6
-DMAC_CFG_RX_MAX=3

Стало:
-DMAC_CFG_TX_DATA_MAX=9 
-DMAC_CFG_TX_MAX=18 
-DMAC_CFG_RX_MAX=9 
____
8. В файле Z-Stack_3.0.2\Projects\zstack\ZMain\TI2530DB\ZMain.c
Было:
  // Initialize the MAC
  ZMacInit();
  
  // Determine the extended address
  zmain_ext_addr();
  
Стало:	
  // Initialize the MAC
  ZMacInit();
  
  ZMacSetTransmitPower(TX_PWR_PLUS_19);

  // Determine the extended address
  zmain_ext_addr();