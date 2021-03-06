/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    NanookV2.cpp
 * @brief   Application entry point.
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "GPS.h"
#include "Uart.h"
#include "Interrupts.h"
#include "MotorController.h"
#include "Commands.h"
#include "GY80.h"
#include "Functions.h"
#include "IMU.h"

#include "Quaternion.h"

void Quaternion::Print(const Quaternion& q){
	PRINTF("%f %f %f %f\r\n",q.w,q.v.x,q.v.y,q.v.z);
}

void Vec3::Print(){
	PRINTF("%f %f %f\r\n",x,y,z);
}


/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

volatile bool botao_sw2;
volatile bool botao_sw3;

//Initialize UART Classes
Uart USB_Uart(UART_USB_PERIPHERAL);
Uart GPS_Uart(UART_GPS_PERIPHERAL);
Uart BT_Uart(UART_BT_PERIPHERAL);

//Initialize Motor Class
MotorController Motor(UART_DRIVER_PERIPHERAL);

//Initialize GPS Class
GPS Gps(&GPS_Uart);

//Initialize Low-Level GY80 Class
GY80 GY;

//Initialize High-Level IMU Class
IMU Imu(&GY);



int main(void) {

    //Interrupt Handles (GAMBIARRA?)
    USB_UART_PTR = &USB_Uart;
    GPS_UART_PTR = &GPS_Uart;
    BT_UART_PTR = &BT_Uart;
    GPS_PTR = &Gps;
    MOTOR_PTR = &Motor;


  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


    Commands::SetUart(&BT_Uart); //BT_UART!
    Commands::SetMotor(&Motor);
    Commands::SetGPS(&Gps);
    Commands::SetIMU(&Imu);

//    USB_UART_PTR->SendString((const uint8_t*)"K64F Ready!\r\n");


    if(GY.Init() == false){
    LED_RED_ON();
    }
 	else{
 	LED_GREEN_ON();
 	}

    delay_us(200000);
    LED_GREEN_OFF();
    LED_RED_OFF();



//    Gps.SetMode(GPS::GLL_MODE);
    Motor.SetMode(MotorController::StopMotor);
    Motor.SetMode(MotorController::ClosedLoop);

    bool uartSwitch;


    while(1){

    	if(uartSwitch){
    		Commands::SetUart(&BT_Uart);

    	} else {
    		Commands::SetUart(&USB_Uart);
    	}

    	uartSwitch = !uartSwitch; // Muda a UART

    	if (Commands::Parse() ){
    		LED_GREEN_TOGGLE();
    	}



    }


    return 0;
}
