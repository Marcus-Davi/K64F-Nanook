/*
 * MotorController.h
 *
 *  Created on: Jul 8, 2019
 *      Author: marcus
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <stdint.h>
#include "fsl_uart.h"


// PI Parameters. Projetado Via MATLAB
#define KpRight 0.001754f
#define KiRight 0.3508f
#define KpLeft 0.00179f
#define KiLeft 0.3419f

#define MOTOR_CONTROL_FREQUENCY_HZ 100
#define MOTOR_CONTROL_SAMPLING_TIME 1/MOTOR_CONTROL_FREQUENCY_HZ

// Aproximação por Reta 28/01/2020
#define ENCODER_RIGHT_ADJUST_B0 0 // Ajustado 28/01/2020
#define ENCODER_RIGHT_ADJUST_B1 0.418383064516129 // Ajustado 28/01/2020

#define ENCODER_LEFT_ADJUST_B0 0	// Ajustado 28/01/2020
#define ENCODER_LEFT_ADJUST_B1 0.451717741935484	// Ajustado 24/01/2020

#define ENCODER_PPR 32750

class MotorController {
private:
	UART_Type* UartPeripheral;

	struct _SpeedMeasurement {
		float Right;
		float Left;
	} SpeedMeasureRPM;

	struct _SpeedReference {
		float Right;
		float Left;
	} SpeedReference;

	struct _PID {

		const float A_Right = KpRight;
		const float  B_Right = KiRight*MOTOR_CONTROL_SAMPLING_TIME - KpRight;

		const float A_Left = KpLeft;
		const float  B_Left = KiLeft*MOTOR_CONTROL_SAMPLING_TIME - KpLeft;

//		float u_d_k1 = 0; //uk1 direito
//		float e_d_k1 = 0; //ek1 direito
//		float u_e_k1 = 0;  //uk1 esquerdo
//		float e_e_k1 = 0; //ek1 esquerdo

	} PID;

	void DriverCommand(float u_right, float u_left);
	int32_t GetRightDelta();
	int32_t GetLeftDelta();
	uint32_t Timeout = 100; //Contagens
	uint32_t Timeout_timer = 0; //Assumindo 100 Hz, 1s de watchdog

public:

	void (MotorController::* Action)();
	enum Side {RightMotor,LeftMotor};
	enum _status {OpenLoop, ClosedLoop,StopMotor} Status;

	struct _encoder {
		int32_t Count_Right;
		int32_t Count_Right_Old;
		int32_t Count_Left;
		int32_t Count_Left_Old;
	} Encoder;




	MotorController(UART_Type* Peripheral);
	void SetMode(_status status);
	void SetSpeedReference(float RightRPM, float LeftRPM);
	void ComputeSpeed();
	float GetSpeed(Side side);

	void LoopPID();
	void Stop();


	//Watchdog Timeout
	void SetTimeout(uint32_t timeout);
	void TimerTick();
	void ClearTimer();
	bool TimerOverflow();
};

#endif /* MOTORCONTROLLER_H_ */
