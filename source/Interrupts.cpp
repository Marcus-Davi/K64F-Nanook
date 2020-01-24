/*
 * interrupts.c
 *
 *  Created on: 10 de ago de 2018
 *      Author: marcus
 */

#include "Interrupts.h"


//Interrupt Handles
Uart* GPS_UART_PTR;
Uart* BT_UART_PTR;
Uart* USB_UART_PTR;
GPS* GPS_PTR;
MotorController* MOTOR_PTR;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


//SW3
void GPIO_A_IRQHANDLER(void){
	botao_sw3 = 1;
	GPIO_PortClearInterruptFlags(GPIOA, 1 << BOARD_BUTTONS_SW3_PIN);
	GPIO_PortToggle(BOARD_LED_RED_GPIO, 1 << BOARD_LED_RED_PIN );
}


//Right Encoder
void GPIO_B_IRQHANDLER(){
	GPIO_PortClearInterruptFlags(ENCODER_PINS_ENC_R_A_GPIO, 1 << ENCODER_PINS_ENC_R_A_PIN );
	if(GPIO_PinRead(ENCODER_PINS_ENC_R_B_GPIO, ENCODER_PINS_ENC_R_B_PIN) == 1)
		MOTOR_PTR->Encoder.Count_Right++;
	else
		MOTOR_PTR->Encoder.Count_Right--;
}

//Left Encoder + Botao2
void GPIO_C_IRQHANDLER(){
	if(GPIO_PortGetInterruptFlags(GPIOC) == 1 << BOARD_BUTTONS_SW2_PIN){
		GPIO_PortClearInterruptFlags(BOARD_BUTTONS_SW2_GPIO, 1 << BOARD_BUTTONS_SW2_PIN);
//		GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1 <<BOARD_LED_BLUE_PIN );
//		botao_sw2 = 1;
//		ROBOT.status = tracking;
//
//		/* GPS SET ORIGIN TEST */
		GPS_PTR->SetOrigin();
//		GPSData.longitude_origin = GPSData.longitude;
//		GPSData.latitude_origin = GPSData.latitude;
//
//		return;
	}
//
//
	GPIO_PortClearInterruptFlags(ENCODER_PINS_ENC_L_A_GPIO, 1 << ENCODER_PINS_ENC_L_A_PIN );
	if(GPIO_PinRead(ENCODER_PINS_ENC_L_B_GPIO, ENCODER_PINS_ENC_L_B_PIN) == 1)
		MOTOR_PTR->Encoder.Count_Left--;
	else
		MOTOR_PTR->Encoder.Count_Left++;
}


void PIT_0_IRQHANDLER(){ //100Hz -> Cálculo da velocidade dos motores
	PIT_ClearStatusFlags(PIT_PERIPHERAL, kPIT_Chnl_0, kPIT_TimerFlag);
	MOTOR_PTR->ComputeSpeed();



	if(MOTOR_PTR->Status == MotorController::StopMotor)
//	if(MOTOR_PTR->Status == MotorController::StopMotor || MOTOR_PTR->TimerOverflow())
		MOTOR_PTR->Stop();//(MOTOR_PTR->*Action)();
	else
		MOTOR_PTR->LoopPID();

	//Update Encoder
	MOTOR_PTR->Encoder.Count_Right_Old = MOTOR_PTR->Encoder.Count_Right;
	MOTOR_PTR->Encoder.Count_Left_Old = MOTOR_PTR->Encoder.Count_Left;

	MOTOR_PTR->TimerTick(); // "Watchdog"
}

void PIT_1_IRQHANDLER(){ //10Hz -> Uso Geral
	PIT_ClearStatusFlags(PIT_PERIPHERAL, kPIT_Chnl_1, kPIT_TimerFlag);

}


//UART BLUETOOTH
void UART_BT_SERIAL_RX_TX_IRQHANDLER(){
	UART_GetStatusFlags(UART_BT_PERIPHERAL);
	uint8_t byte = UART_ReadByte(UART_BT_PERIPHERAL);
	BT_UART_PTR->InterruptHandle(byte);

}

//UART GPS
void UART_GPS_SERIAL_RX_TX_IRQHANDLER(){
	UART_GetStatusFlags(UART_GPS_PERIPHERAL);
	uint8_t byte = UART_ReadByte(UART_GPS_PERIPHERAL);
	GPS_UART_PTR->InterruptHandle(byte);

}

void UART_USB_SERIAL_RX_TX_IRQHANDLER(){
	UART_GetStatusFlags(UART_USB_PERIPHERAL);
	uint8_t byte = UART_ReadByte(UART_USB_PERIPHERAL);
	USB_UART_PTR->InterruptHandle(byte);
}


#if defined(__cplusplus)
}
#endif
