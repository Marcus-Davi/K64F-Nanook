/*
 * Commands.hpp
 *
 *  Created on: 22 de mai de 2019
 *      Author: marcus
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#include <stdint.h>

#include "Uart.h"
#include "MotorController.h"
#include "GPS.h"
#include "IMU.h"

class Commands {
private:


	static const char* CMDS[];
	static const uint8_t N_Commands = 8; //Number of Commands
	//Classes
	static MotorController* Motor;
	static Uart* UART;
	static GPS* Gps;
	static IMU* Imu;


public:



	static inline void SetUart(Uart* uart_in){
		Commands::UART = uart_in;
	}

	static inline void SetMotor(MotorController* MOTOR){
		Motor = MOTOR;
	}

	static inline void SetGPS(GPS* gps){
		Gps = gps;
	}

	static inline void SetIMU(IMU* imu){
		Imu = imu;
	}

	//funciona bem a 1.75+ KHz !
	static bool Parse(){
		static char buffer_out[200];
		char* Pt;
		int k;
		bool valid = false;
		static uint8_t* Buffer = UART->GetBuffer();

		//Please make sure terminator is asserted.

		if(strchr((char*)Buffer,'\r')==0)
			return valid; //sem token

		for(k=0;k<N_Commands;k++){
			Pt = strstr((char*)Buffer,CMDS[k]);
			if(Pt){
				break;
			}
		}

		switch(k) {
		case 0:{ //S?
			Imu->GetInertialRaw();
			float vd = Motor->GetSpeed(MotorController::RightMotor);
			float ve = Motor->GetSpeed(MotorController::LeftMotor);
			double x = Gps->GetX();
			double y = Gps->GetY();
			sprintf(buffer_out,"%d %d %d %d %d %d %d %d %d %f %f %f %f\r",
					Imu->IAcc.X,Imu->IAcc.Y,Imu->IAcc.Z,
					Imu->IGyr.X,Imu->IGyr.Y,Imu->IGyr.Z,
					Imu->IMag.X,Imu->IMag.Y,Imu->IMag.Z,vd,ve,x,y);
			UART->SendString((uint8_t*)buffer_out);
			valid = true;
			break;
		}

		case 1: //I?
			Imu->GetInertialRaw();
			sprintf(buffer_out,"%d %d %d %d %d %d %d %d %d\r",
					Imu->IAcc.X,Imu->IAcc.Y,Imu->IAcc.Z,
					Imu->IGyr.X,Imu->IGyr.Y,Imu->IGyr.Z,
					Imu->IMag.X,Imu->IMag.Y,Imu->IMag.Z);
			UART->SendString((uint8_t*)buffer_out);
			valid = true;
			break;
		case 2:{ // A?
			float angle = atan2(-Imu->IMag.Y +103 ,Imu->IMag.X+161);
			sprintf(buffer_out,"%f",angle);
			UART->SendString((uint8_t*)buffer_out);
			valid = true;
			break;

		}
		case 3:{ //M!
			char* token = strchr((char*)Buffer,(char)'!');
			float Vd,Ve;
			Vd = atoi((char*) ++token); //convere o primeiro dado
			token = strchr((char*)Buffer, (char)','); //procura a virgula
			Ve = (atoi((char*) ++token));
			Motor->SetSpeedReference((float) Vd, (float)Ve);
			valid = true;
			break;

		}
		case 4:{ //M?
			float vd = Motor->GetSpeed(MotorController::RightMotor);
			float ve = Motor->GetSpeed(MotorController::LeftMotor);
			sprintf(buffer_out,"%f %f\r",vd,ve);
			UART->SendString((uint8_t*)buffer_out);
			valid = true;
		//	return VELOCITY_QUERY;
			break;
		}

		case 5:{ //G?
			float latitude = Gps->GetLatitude();
			float longitude = Gps->GetLongitude();
			sprintf(buffer_out,"%f %f\r",latitude,longitude);
			UART->SendString((uint8_t*)buffer_out);
			valid = true;
		}
		case 6:{ //T
			Gps->SetOrigin();
			valid = true;
		}

		default:
		//	return NOP;
			break;

		}

		UART->ClearBuffer();
		return valid;
	}





};

//Gambiarras
MotorController* Commands::Motor = 0;
Uart* Commands::UART= 0;
GPS* Commands::Gps = 0;
IMU* Commands::Imu = 0;
const char* Commands::CMDS[] = {"S?\r","I?\r","A?\r","M! ","M?\r","G?\r","T!\r"};
#endif /* INC_COMMANDS_H_ */
