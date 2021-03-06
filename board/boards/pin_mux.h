/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTB21 (number 67), D12[3]/LEDRGB_BLUE
  @{ */
#define BOARD_LED_BLUE_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_LED_BLUE_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_LED_BLUE_PIN 21U    /*!<@brief PORTB pin index: 21 */
                                  /* @} */

/*! @name PORTB22 (number 68), D12[1]/LEDRGB_RED
  @{ */
#define BOARD_LED_RED_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_LED_RED_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_LED_RED_PIN 22U    /*!<@brief PORTB pin index: 22 */
                                 /* @} */

/*! @name PORTE26 (number 33), J2[1]/D12[4]/LEDRGB_GREEN
  @{ */
#define BOARD_LED_GREEN_GPIO GPIOE /*!<@brief GPIO device name: GPIOE */
#define BOARD_LED_GREEN_PORT PORTE /*!<@brief PORT device name: PORTE */
#define BOARD_LED_GREEN_PIN 26U    /*!<@brief PORTE pin index: 26 */
                                   /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLEDs(void);

/*! @name PORTC6 (number 78), U8[11]/SW2
  @{ */
#define BOARD_BUTTONS_SW2_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_BUTTONS_SW2_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_BUTTONS_SW2_PIN 6U     /*!<@brief PORTC pin index: 6 */
                                     /* @} */

/*! @name PORTA4 (number 38), SW3
  @{ */
#define BOARD_BUTTONS_SW3_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
#define BOARD_BUTTONS_SW3_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_BUTTONS_SW3_PIN 4U     /*!<@brief PORTA pin index: 4 */
                                     /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_Buttons(void);

#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*! @name PORTB17 (number 63), U10[1]/UART0_TX
  @{ */
#define BOARD_DEBUG_UART_TX_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_DEBUG_UART_TX_PIN 17U    /*!<@brief PORTB pin index: 17 */
                                       /* @} */

/*! @name PORTB16 (number 62), U7[4]/UART0_RX
  @{ */
#define BOARD_DEBUG_UART_RX_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_DEBUG_UART_RX_PIN 16U    /*!<@brief PORTB pin index: 16 */
                                       /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UART(void);

/*! @name PORTC15 (number 87), J199[4]/BT_RX
  @{ */
#define BT_PINS_BT_TX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BT_PINS_BT_TX_PIN 15U    /*!<@brief PORTC pin index: 15 */
                                 /* @} */

/*! @name PORTC14 (number 86), J199[3]/BT_TX
  @{ */
#define BT_PINS_BT_RX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BT_PINS_BT_RX_PIN 14U    /*!<@brief PORTC pin index: 14 */
                                 /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BT_PINS(void);

/*! @name PORTB18 (number 64), J1[1]
  @{ */
#define ENCODER_PINS_ENC_R_A_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define ENCODER_PINS_ENC_R_A_PORT PORTB /*!<@brief PORT device name: PORTB */
#define ENCODER_PINS_ENC_R_A_PIN 18U    /*!<@brief PORTB pin index: 18 */
                                        /* @} */

/*! @name PORTB19 (number 65), J1[3]
  @{ */
#define ENCODER_PINS_ENC_R_B_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define ENCODER_PINS_ENC_R_B_PORT PORTB /*!<@brief PORT device name: PORTB */
#define ENCODER_PINS_ENC_R_B_PIN 19U    /*!<@brief PORTB pin index: 19 */
                                        /* @} */

/*! @name PORTC8 (number 80), J1[7]
  @{ */
#define ENCODER_PINS_ENC_L_A_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define ENCODER_PINS_ENC_L_A_PORT PORTC /*!<@brief PORT device name: PORTC */
#define ENCODER_PINS_ENC_L_A_PIN 8U     /*!<@brief PORTC pin index: 8 */
                                        /* @} */

/*! @name PORTC1 (number 71), J1[5]
  @{ */
#define ENCODER_PINS_ENC_L_B_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define ENCODER_PINS_ENC_L_B_PORT PORTC /*!<@brief PORT device name: PORTC */
#define ENCODER_PINS_ENC_L_B_PIN 1U     /*!<@brief PORTC pin index: 1 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void ENCODER_PINS(void);

#define SOPT5_UART1TXSRC_UART_TX 0x00u /*!<@brief UART 1 transmit data source select: UART1_TX pin */

/*! @name PORTC3 (number 73), J1[16]
  @{ */
#define DRIVER_PINS_MOTOR_RX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define DRIVER_PINS_MOTOR_RX_PIN 3U     /*!<@brief PORTC pin index: 3 */
                                        /* @} */

/*! @name PORTC4 (number 76), J2[4]
  @{ */
#define DRIVER_PINS_MOTOR_TX_PORT PORTC /*!<@brief PORT device name: PORTC */
#define DRIVER_PINS_MOTOR_TX_PIN 4U     /*!<@brief PORTC pin index: 4 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void DRIVER_PINS(void);

/*! @name PORTC10 (number 82), J4[12]
  @{ */
#define IMU_PINS_IMU_SCL_PORT PORTC /*!<@brief PORT device name: PORTC */
#define IMU_PINS_IMU_SCL_PIN 10U    /*!<@brief PORTC pin index: 10 */
                                    /* @} */

/*! @name PORTC11 (number 83), J4[10]
  @{ */
#define IMU_PINS_IMU_SDA_PORT PORTC /*!<@brief PORT device name: PORTC */
#define IMU_PINS_IMU_SDA_PIN 11U    /*!<@brief PORTC pin index: 11 */
                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void IMU_PINS(void);

/*! @name PORTC16 (number 90), J1[2]
  @{ */
#define GPS_PINS_TMR_1588_0_PORT PORTC /*!<@brief PORT device name: PORTC */
#define GPS_PINS_TMR_1588_0_PIN 16U    /*!<@brief PORTC pin index: 16 */
                                       /* @} */

/*! @name PORTC17 (number 91), J1[4]
  @{ */
#define GPS_PINS_TMR_1588_1_PORT PORTC /*!<@brief PORT device name: PORTC */
#define GPS_PINS_TMR_1588_1_PIN 17U    /*!<@brief PORTC pin index: 17 */
                                       /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void GPS_PINS(void);

/*! @name PORTB9 (number 57), J1[6]
  @{ */
#define BOARD_PINS_DEBUG_PIN_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_PINS_DEBUG_PIN_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_PINS_DEBUG_PIN_PIN 9U     /*!<@brief PORTB pin index: 9 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_Pins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
