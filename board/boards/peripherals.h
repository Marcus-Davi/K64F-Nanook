/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_pit.h"
#include "fsl_uart.h"
#include "fsl_clock.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Alias for GPIOA peripheral */
#define GPIO_A_GPIO GPIOA
/* Alias for PORTA */
#define GPIO_A_PORT PORTA
/* GPIO_A interrupt vector ID (number). */
#define GPIO_A_IRQN PORTA_IRQn
/* GPIO_A interrupt handler identifier. */
#define GPIO_A_IRQHANDLER PORTA_IRQHandler
/* Alias for GPIOB peripheral */
#define GPIO_B_GPIO GPIOB
/* Alias for PORTB */
#define GPIO_B_PORT PORTB
/* GPIO_B interrupt vector ID (number). */
#define GPIO_B_IRQN PORTB_IRQn
/* GPIO_B interrupt vector priority. */
#define GPIO_B_IRQ_PRIORITY 1
/* GPIO_B interrupt handler identifier. */
#define GPIO_B_IRQHANDLER PORTB_IRQHandler
/* Alias for GPIOC peripheral */
#define GPIO_C_GPIO GPIOC
/* Alias for PORTC */
#define GPIO_C_PORT PORTC
/* GPIO_C interrupt vector ID (number). */
#define GPIO_C_IRQN PORTC_IRQn
/* GPIO_C interrupt vector priority. */
#define GPIO_C_IRQ_PRIORITY 1
/* GPIO_C interrupt handler identifier. */
#define GPIO_C_IRQHANDLER PORTC_IRQHandler
/* BOARD_InitPeripherals defines for I2C1 */
/* Definition of peripheral ID */
#define I2C_1_PERIPHERAL I2C1
/* Definition of the clock source */
#define I2C_1_CLOCK_SOURCE I2C1_CLK_SRC
/* Definition of the clock source frequency */
#define I2C_1_CLK_FREQ CLOCK_GetFreq(I2C_1_CLOCK_SOURCE)
/* BOARD_InitPeripherals defines for PIT */
/* Definition of peripheral ID. */
#define PIT_PERIPHERAL PIT
/* Definition of clock source. */
#define PIT_CLOCK_SOURCE kCLOCK_BusClk
/* Definition of clock source frequency. */
#define PIT_CLK_FREQ 60000000UL
/* Definition of ticks count for channel 0 - deprecated. */
#define PIT_0_TICKS 599999U
/* Definition of ticks count for channel 1 - deprecated. */
#define PIT_1_TICKS 5999999U
/* Definition of ticks count for channel 2 - deprecated. */
#define PIT_2_TICKS 59999U
/* Definition of channel number for channel 0. */
#define PIT_0 kPIT_Chnl_0
/* Definition of channel number for channel 1. */
#define PIT_1 kPIT_Chnl_1
/* Definition of channel number for channel 2. */
#define PIT_2 kPIT_Chnl_2
/* PIT interrupt vector ID (number). */
#define PIT_0_IRQN PIT0_IRQn
/* PIT interrupt handler identifier. */
#define PIT_0_IRQHANDLER PIT0_IRQHandler
/* PIT interrupt vector ID (number). */
#define PIT_1_IRQN PIT1_IRQn
/* PIT interrupt handler identifier. */
#define PIT_1_IRQHANDLER PIT1_IRQHandler
/* Definition of peripheral ID */
#define UART_BT_PERIPHERAL UART4
/* Definition of the clock source frequency */
#define UART_BT_CLOCK_SOURCE CLOCK_GetFreq(UART4_CLK_SRC)
/* UART_BT interrupt vector ID (number). */
#define UART_BT_SERIAL_RX_TX_IRQN UART4_RX_TX_IRQn
/* UART_BT interrupt vector priority. */
#define UART_BT_SERIAL_RX_TX_IRQ_PRIORITY 0
/* UART_BT interrupt handler identifier. */
#define UART_BT_SERIAL_RX_TX_IRQHANDLER UART4_RX_TX_IRQHandler
/* UART_BT interrupt vector ID (number). */
#define UART_BT_SERIAL_ERROR_IRQN UART4_ERR_IRQn
/* UART_BT interrupt handler identifier. */
#define UART_BT_SERIAL_ERROR_IRQHANDLER UART4_ERR_IRQHandler
/* Definition of peripheral ID */
#define UART_DRIVER_PERIPHERAL UART1
/* Definition of the clock source frequency */
#define UART_DRIVER_CLOCK_SOURCE CLOCK_GetFreq(UART1_CLK_SRC)
/* Definition of peripheral ID */
#define UART_GPS_PERIPHERAL UART3
/* Definition of the clock source frequency */
#define UART_GPS_CLOCK_SOURCE CLOCK_GetFreq(UART3_CLK_SRC)
/* UART_GPS interrupt vector ID (number). */
#define UART_GPS_SERIAL_RX_TX_IRQN UART3_RX_TX_IRQn
/* UART_GPS interrupt handler identifier. */
#define UART_GPS_SERIAL_RX_TX_IRQHANDLER UART3_RX_TX_IRQHandler
/* UART_GPS interrupt vector ID (number). */
#define UART_GPS_SERIAL_ERROR_IRQN UART3_ERR_IRQn
/* UART_GPS interrupt handler identifier. */
#define UART_GPS_SERIAL_ERROR_IRQHANDLER UART3_ERR_IRQHandler
/* Definition of peripheral ID */
#define UART_USB_PERIPHERAL UART0
/* Definition of the clock source frequency */
#define UART_USB_CLOCK_SOURCE CLOCK_GetFreq(UART0_CLK_SRC)
/* UART_USB interrupt vector ID (number). */
#define UART_USB_SERIAL_RX_TX_IRQN UART0_RX_TX_IRQn
/* UART_USB interrupt handler identifier. */
#define UART_USB_SERIAL_RX_TX_IRQHANDLER UART0_RX_TX_IRQHandler
/* UART_USB interrupt vector ID (number). */
#define UART_USB_SERIAL_ERROR_IRQN UART0_ERR_IRQn
/* UART_USB interrupt handler identifier. */
#define UART_USB_SERIAL_ERROR_IRQHANDLER UART0_ERR_IRQHandler

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const i2c_master_config_t I2C_1_config;
extern const pit_config_t PIT_config;
extern const uart_config_t UART_BT_config;
extern const uart_config_t UART_DRIVER_config;
extern const uart_config_t UART_GPS_config;
extern const uart_config_t UART_USB_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
