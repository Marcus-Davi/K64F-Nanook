/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v6.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 6.0.1
board: FRDM-K64F
functionalGroups:
- name: BOARD_InitPeripherals
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'msg'
- type_id: 'msg_6e2baaf3b97dbeef01c0043275f9a0e7'
- global_messages: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * GPIO_B initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_B'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_priority: 'true'
      - priority: '1'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_B_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTB_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTB_IRQn, GPIO_B_IRQ_PRIORITY);
  /* Enable interrupt PORTB_IRQn request in the NVIC */
  EnableIRQ(PORTB_IRQn);
}

/***********************************************************************************************************************
 * GPIO_C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_C'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_priority: 'true'
      - priority: '1'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_C_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTC_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTC_IRQn, GPIO_C_IRQ_PRIORITY);
  /* Enable interrupt PORTC_IRQn request in the NVIC */
  EnableIRQ(PORTC_IRQn);
}

/***********************************************************************************************************************
 * UART_USB initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_USB'
- type: 'uart'
- mode: 'interrupts'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
  - interruptsCfg:
    - interrupts: 'kUART_RxDataRegFullInterruptEnable'
    - interrupt_vectors:
      - enable_rx_tx_irq: 'true'
      - interrupt_rx_tx:
        - IRQn: 'UART0_RX_TX_IRQn'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
      - enable_err_irq: 'false'
      - interrupt_err:
        - IRQn: 'UART0_ERR_IRQn'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_USB_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_USB_init(void) {
  UART_Init(UART_USB_PERIPHERAL, &UART_USB_config, UART_USB_CLOCK_SOURCE);
  UART_EnableInterrupts(UART_USB_PERIPHERAL, kUART_RxDataRegFullInterruptEnable);
  /* Enable interrupt UART0_RX_TX_IRQn request in the NVIC */
  EnableIRQ(UART_USB_SERIAL_RX_TX_IRQN);
}

/***********************************************************************************************************************
 * UART_DRIVER initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_DRIVER'
- type: 'uart'
- mode: 'polling'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART1'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '9600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_DRIVER_config = {
  .baudRate_Bps = 9600,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_DRIVER_init(void) {
  UART_Init(UART_DRIVER_PERIPHERAL, &UART_DRIVER_config, UART_DRIVER_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- type_id: 'pit_a4782ba5223c8a2527ba91aeb2bc4159'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'false'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'BOARD_BootClockRUN'
    - channels:
      - 0:
        - channelNumber: '0'
        - enableChain: 'false'
        - timerPeriod: '100Hz'
        - startTimer: 'true'
        - enableInterrupt: 'true'
        - interrupt:
          - IRQn: 'PIT0_IRQn'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 1:
        - channelNumber: '1'
        - enableChain: 'false'
        - timerPeriod: '10Hz'
        - startTimer: 'true'
        - enableInterrupt: 'true'
        - interrupt:
          - IRQn: 'PIT1_IRQn'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 2:
        - channelNumber: '2'
        - enableChain: 'false'
        - timerPeriod: '1000Hz'
        - startTimer: 'false'
        - enableInterrupt: 'false'
        - interrupt:
          - IRQn: 'PIT0_IRQn'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = false
};

void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
  /* Set channel 0 period to 10 ms (600000 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, kPIT_Chnl_0, PIT_0_TICKS);
  /* Set channel 1 period to 100 ms (6000000 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, kPIT_Chnl_1, PIT_1_TICKS);
  /* Set channel 2 period to 1 ms (60000 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, kPIT_Chnl_2, PIT_2_TICKS);
  /* Enable interrupts from channel 0. */
  PIT_EnableInterrupts(PIT_PERIPHERAL, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
  /* Enable interrupts from channel 1. */
  PIT_EnableInterrupts(PIT_PERIPHERAL, kPIT_Chnl_1, kPIT_TimerInterruptEnable);
  /* Enable interrupt PIT_0_IRQN request in the NVIC */
  EnableIRQ(PIT_0_IRQN);
  /* Enable interrupt PIT_1_IRQN request in the NVIC */
  EnableIRQ(PIT_1_IRQN);
  /* Start channel 0. */
  PIT_StartTimer(PIT_PERIPHERAL, kPIT_Chnl_0);
  /* Start channel 1. */
  PIT_StartTimer(PIT_PERIPHERAL, kPIT_Chnl_1);
}

/***********************************************************************************************************************
 * UART_BT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_BT'
- type: 'uart'
- mode: 'interrupts'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART4'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
  - interruptsCfg:
    - interrupts: 'kUART_RxDataRegFullInterruptEnable'
    - interrupt_vectors:
      - enable_rx_tx_irq: 'true'
      - interrupt_rx_tx:
        - IRQn: 'UART4_RX_TX_IRQn'
        - enable_priority: 'true'
        - priority: '0'
        - enable_custom_name: 'false'
      - enable_err_irq: 'false'
      - interrupt_err:
        - IRQn: 'UART4_ERR_IRQn'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_BT_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_BT_init(void) {
  UART_Init(UART_BT_PERIPHERAL, &UART_BT_config, UART_BT_CLOCK_SOURCE);
  UART_EnableInterrupts(UART_BT_PERIPHERAL, kUART_RxDataRegFullInterruptEnable);
  /* Interrupt vector UART4_RX_TX_IRQn priority settings in the NVIC */
  NVIC_SetPriority(UART_BT_SERIAL_RX_TX_IRQN, UART_BT_SERIAL_RX_TX_IRQ_PRIORITY);
  /* Enable interrupt UART4_RX_TX_IRQn request in the NVIC */
  EnableIRQ(UART_BT_SERIAL_RX_TX_IRQN);
}

/***********************************************************************************************************************
 * I2C_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C_1'
- type: 'i2c'
- mode: 'I2C_Polling'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C1'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '350000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t I2C_1_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 350000,
  .glitchFilterWidth = 0
};

void I2C_1_init(void) {
  /* Initialization function */
  I2C_MasterInit(I2C_1_PERIPHERAL, &I2C_1_config, I2C_1_CLK_FREQ);
}

/***********************************************************************************************************************
 * GPIO_A initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIO_A'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void GPIO_A_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
  /* Enable interrupt PORTA_IRQn request in the NVIC */
  EnableIRQ(PORTA_IRQn);
}

/***********************************************************************************************************************
 * UART_GPS initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_GPS'
- type: 'uart'
- mode: 'interrupts'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART3'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '57600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
  - interruptsCfg:
    - interrupts: 'kUART_RxDataRegFullInterruptEnable'
    - interrupt_vectors:
      - enable_rx_tx_irq: 'true'
      - interrupt_rx_tx:
        - IRQn: 'UART3_RX_TX_IRQn'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
      - enable_err_irq: 'false'
      - interrupt_err:
        - IRQn: 'UART3_ERR_IRQn'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_GPS_config = {
  .baudRate_Bps = 57600,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

void UART_GPS_init(void) {
  UART_Init(UART_GPS_PERIPHERAL, &UART_GPS_config, UART_GPS_CLOCK_SOURCE);
  UART_EnableInterrupts(UART_GPS_PERIPHERAL, kUART_RxDataRegFullInterruptEnable);
  /* Enable interrupt UART3_RX_TX_IRQn request in the NVIC */
  EnableIRQ(UART_GPS_SERIAL_RX_TX_IRQN);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPIO_B_init();
  GPIO_C_init();
  UART_USB_init();
  UART_DRIVER_init();
  PIT_init();
  UART_BT_init();
  I2C_1_init();
  GPIO_A_init();
  UART_GPS_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
