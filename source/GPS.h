/*
 * GPS.hpp
 *
 *  Created on: 16 de mai de 2019
 *      Author: marcus
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "Uart.h"
#include <math.h>

#define deg2rad(deg) (deg*M_PI/180)
#define a 6378137.0f //km

//Double precision for GPS DATA is RECOMMENDED

class GPS {

private:

   static unsigned const char AlwaysLocateMode[];
   static unsigned const char StandbyMode[];
   static unsigned const char Set9600Baud[];
   static unsigned const char SetHertz[];
   static unsigned const char GGAMode[];
   static unsigned const char GLLMode[];

   //Raw Data
   double Latitude;
   double Longitude;
   double Latitude_Origin;
   double Longitude_Origin;
	//GeoNorth
   double X;
   double Y;
	//for UTM
   double Easting;
   double Northing;

	//
	Uart* UART;

public:

	enum MODE {
		GLL_MODE,
		DEFAULT_MODE
	};

	GPS(Uart* GPS_UART);
	void SetMode(MODE mode);
	void SetOrigin(float Lat, float Long);
	void SetOrigin();
	double GetX();
	double GetY();
	double GetLatitude();
	double GetLongitude();
	bool GetLatLong();
	bool WaitData();

};

#endif /* INC_GPS_H_ */
