/*
 * IMU.h
 *
 *  Created on: 11 de jul de 2019
 *      Author: marcus
 */

#ifndef IMU_H_
#define IMU_H_

#include "GY80.h" //SENSOR INCLUSION !!


#include "math.h"


class IMU {
	struct FVector3 {
		float X;
		float Y;
		float Z;
	};

	struct IVector3 {
		int16_t X;
		int16_t Y;
		int16_t Z;
	};

private:
	GY80* Sensor;

public:

	static constexpr float Gravity = 9.80665f;
	static constexpr float Deg2Rad = M_PI/180.0f;

	FVector3 Acc;
	FVector3 Gyr;
	FVector3 Mag;

	IVector3 IAcc;
	IVector3 IGyr;
	IVector3 IMag;



	IMU(GY80* sensor);


	void GetAccelerations();
	void GetAngularVelocities();
	void GetMagnetic();

	void GetInertial();
	void GetInertialRaw();
	float GetMagYaw();



};


#endif /* IMU_H_ */
