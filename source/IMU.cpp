/*
 * IMU.cpp
 *
 *  Created on: 11 de jul de 2019
 *      Author: marcus
 */

#include <IMU.h>

IMU::IMU(GY80* sensor) {
	Sensor = sensor;
	// TODO Auto-generated constructor stub
}


void IMU::GetAccelerations(){
	Sensor->ReadAcc();
	Acc.X = (float)(Sensor->Imu.Acc.X - Sensor->Imu.AccOffset.X)*GY_ACC_RESOLUTION*Gravity;
	Acc.Y = (float)(Sensor->Imu.Acc.Y - Sensor->Imu.AccOffset.Y)*GY_ACC_RESOLUTION*Gravity;
	Acc.Z = (float)(Sensor->Imu.Acc.Z - Sensor->Imu.AccOffset.Z)*GY_ACC_RESOLUTION*Gravity;
}

void IMU::GetAngularVelocities(){
	Sensor->ReadGyr();
	Gyr.X = (float)(Sensor->Imu.Gyr.X - Sensor->Imu.GyroOffset.X)*GY_GYR_RESOLUTION*Deg2Rad;
	Gyr.Y = (float)(Sensor->Imu.Gyr.Y - Sensor->Imu.GyroOffset.Y)*GY_GYR_RESOLUTION*Deg2Rad;
	Gyr.Z = (float)(Sensor->Imu.Gyr.Z - Sensor->Imu.GyroOffset.Z)*GY_GYR_RESOLUTION*Deg2Rad;
}

void IMU::GetMagnetic(){
	Sensor->ReadMag();
	Mag.X = (float)(Sensor->Imu.Mag.X - Sensor->Imu.MagOffset.X)*GY_MAG_RESOLUTION;
	Mag.Y = (float)(Sensor->Imu.Mag.Y - Sensor->Imu.MagOffset.Y)*GY_MAG_RESOLUTION;
	Mag.Z = (float)(Sensor->Imu.Mag.Z - Sensor->Imu.MagOffset.Z)*GY_MAG_RESOLUTION;
}

void IMU::GetInertial(){
	GetAccelerations();
	GetAngularVelocities();
	GetMagnetic();
}

void IMU::GetInertialRaw(){
	Sensor->ReadAll();
	IAcc.X = Sensor->Imu.Acc.X;
	IAcc.Y = Sensor->Imu.Acc.Y;
	IAcc.Z = Sensor->Imu.Acc.Z;

	IGyr.X = Sensor->Imu.Gyr.X;
	IGyr.Y = Sensor->Imu.Gyr.Y;
	IGyr.Z = Sensor->Imu.Gyr.Z;

	IMag.X = Sensor->Imu.Mag.X;
	IMag.Y = Sensor->Imu.Mag.Y;
	IMag.Z = Sensor->Imu.Mag.Z;

}

float IMU::GetMagYaw(){
	GetMagnetic();
		return atan2(-Mag.Y,Mag.X); //[-pi < atan2 < pi ]
}
