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

#define KpRight 0.001754f
#define KiRight 0.3508f
#define KpLeft 0.00179f
#define KiLeft 0.3419f

#define MOTOR_CONTROL_FREQUENCY_HZ 100
#define MOTOR_CONTROL_SAMPLING_TIME 1/MOTOR_CONTROL_FREQUENCY_HZ
#define ENCODER_RIGHT_ADJUST 0.4137 // Ajuste do encoder direito | old = 0.4137
#define ENCODER_LEFT_ADJUST 0.425	// Ajuste do encoder esquerdo | old = 0.4151
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
	} PID;

	void DriverCommand(float u_right, float u_left);
	int32_t GetRightDelta();
	int32_t GetLeftDelta();
	uint32_t Timeout; //Contagens
	uint32_t Timeout_timer = 100; //Assumindo 100 Hz, 1s de watchdog

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


	//Timeout
	void SetTimeout(uint32_t timeout);
	void TimerTick();
	void ClearTimer();
	bool TimerOverflow();
};

#endif /* MOTORCONTROLLER_H_ */
