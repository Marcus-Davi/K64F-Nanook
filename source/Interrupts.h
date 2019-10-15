/*
 * interrupts.h
 *
 *  Created on: 10 de ago de 2018
 *      Author: marcus
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_



extern volatile bool botao_sw3;
extern volatile bool botao_sw2;


#define PIT1_PERIOD_MS 100

#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "pin_mux.h"
#include "peripherals.h"

#include "Uart.h"
#include "GPS.h"
#include "MotorController.h"
//Interrupt Handles
extern Uart* GPS_UART_PTR;
extern Uart* BT_UART_PTR;
extern Uart* USB_UART_PTR;
extern GPS* GPS_PTR;
extern MotorController* MOTOR_PTR;
//#include "bluetooth.h"
//#include "motor_driver.h"
//#include "robot.h"
//#include "gps.h"


#endif /* INTERRUPTS_H_ */
