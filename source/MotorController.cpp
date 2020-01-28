/*
 * MotorController.cpp
 *
 *  Created on: Jul 8, 2019
 *      Author: marcus
 */

#include <MotorController.h>




MotorController::MotorController(UART_Type* Peripheral) {
	// TODO Auto-generated constructor stub
	//Action = &MotorController::Stop;
	UartPeripheral = Peripheral;
	Status = StopMotor;

}

void MotorController::SetMode(_status status){
	Status = status;
}

int32_t MotorController::GetRightDelta(){
	int32_t delta = Encoder.Count_Right - Encoder.Count_Right_Old;
	// Tratamento da diferença caso haja estouro do contador
	if ((Encoder.Count_Right & 0x80000000) != (Encoder.Count_Right_Old & 0x80000000))
	{
		if (Encoder.Count_Right > Encoder.Count_Right_Old)
			delta = delta - 0xffffffff;
		else
			delta = 0xffffffff + delta;
	}
	return delta;
}

int32_t MotorController::GetLeftDelta(){
	int32_t delta = Encoder.Count_Left - Encoder.Count_Left_Old;
	// Tratamento da diferença caso haja estouro do contador
	if ((Encoder.Count_Left & 0x80000000) != (Encoder.Count_Left_Old & 0x80000000))
	{
		if (Encoder.Count_Left > Encoder.Count_Left_Old)
			delta = delta - 0xffffffff;
		else
			delta = 0xffffffff + delta;
	}
	return delta;
}


void MotorController::DriverCommand(float u_right, float u_left){
	/* Conversão dos sinais de controle [-1.0, 1.0] para o formato aceito pelo
	** driver dos motores [0, 255] e envia dos comandos via serial
	*/

	// Conversão

	uint8_t right_vel_cmd = (uint8_t)((float)((u_right + 1) * 63 + 1));	// [0, 127]
	uint8_t left_vel_cmd = (uint8_t)((float)((u_left + 1) * 63 + 129));	// [129, 255]

	// Envio do comando referente ao motor direito

	UART_WriteBlocking(UartPeripheral, &right_vel_cmd, 1);
	UART_WriteBlocking(UartPeripheral, &left_vel_cmd, 1);


}

void MotorController::LoopPID(){
	//Inicializa estruturas do controlador
	static float u_d_k1 = 0; //uk1 direito
	static float e_d_k1 = 0; //ek1 direito
	static float u_e_k1 = 0;  //uk1 esquerdo
	static float e_e_k1 = 0; //ek1 esquerdo

	float e_d = SpeedReference.Right - SpeedMeasureRPM.Right;
	float e_e = SpeedReference.Left - SpeedMeasureRPM.Left;

	float u_d = u_d_k1 + e_d*PID.A_Right + e_d_k1*PID.B_Right;
	float u_e = u_e_k1 + e_e*PID.A_Left + e_e_k1*PID.B_Left;

	//Saturation
	if(u_d > 1.0)
		u_d = 1.0;
	else if(u_d < -1.0)
		u_d = -1.0;

	if(u_e > 1.0)
		u_e = 1.0;
	else if(u_e < -1.0)
		u_e = -1.0;

	DriverCommand(u_d, u_e);

//	Atualiza
	u_d_k1 = u_d;
	u_e_k1 = u_e;
	e_d_k1 = e_d;
	e_e_k1 = e_e;
}

//Vel em RPM
void MotorController::ComputeSpeed(){
	float deltaR = GetRightDelta();
	float deltaL = GetLeftDelta();
//		SpeedMeasureRPM.Right = ENCODER_RIGHT_ADJUST * (deltaR * 60.0 * MOTOR_CONTROL_FREQUENCY_HZ) / ENCODER_PPR;
//		SpeedMeasureRPM.Left = ENCODER_LEFT_ADJUST * (deltaL * 60.0 * MOTOR_CONTROL_FREQUENCY_HZ) / ENCODER_PPR;

	//Atualizado 28/01/2020
	SpeedMeasureRPM.Right = (ENCODER_RIGHT_ADJUST_B1 * (deltaR * 60.0 * MOTOR_CONTROL_FREQUENCY_HZ) / ENCODER_PPR) + ENCODER_RIGHT_ADJUST_B0;
	SpeedMeasureRPM.Left = (ENCODER_LEFT_ADJUST_B1 * (deltaL * 60.0 * MOTOR_CONTROL_FREQUENCY_HZ) / ENCODER_PPR) + ENCODER_LEFT_ADJUST_B0;
}

float MotorController::GetSpeed(Side side){
	if (side == RightMotor)
		return SpeedMeasureRPM.Right;
	else //left
		return SpeedMeasureRPM.Left;
}

//Right,Left
void MotorController::SetSpeedReference(float RightRPM, float LeftRPM){
	SpeedReference.Right = RightRPM;
	SpeedReference.Left = LeftRPM;
}

void MotorController::Stop(){
	DriverCommand(0,0);
}

//Timeout

void MotorController::TimerTick(){
	Timeout_timer++;
}

void MotorController::SetTimeout(uint32_t timeout){
	Timeout = timeout;
}

void MotorController::ClearTimer(){
	Timeout_timer = 0;
}

bool MotorController::TimerOverflow(){
	if(Timeout_timer > Timeout)
		return true;
	return false;
}

