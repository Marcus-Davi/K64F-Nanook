/*
 * GPS.cpp
 *
 *  Created on: 16 de mai de 2019
 *      Author: marcus
 */

#include "GPS.h"

   const unsigned char GPS::AlwaysLocateMode[]={"$PMTK225,8*23\r\n"};
   const unsigned char GPS::StandbyMode[] = {"$PMTK161,0*28\x0D\x0A"};
   const unsigned char GPS::Set9600Baud[] = {"$PMTK251,9600*17\r\n"};
   const unsigned char GPS::SetHertz[] = {"$PMTK220,1000*1F\r\n"};
   const unsigned char GPS::GGAMode[] = {"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};
   const unsigned char GPS::GLLMode[] = {"$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};

GPS::GPS(Uart* GPS_UART) {
	this->UART = GPS_UART;
}

void GPS::SetMode(MODE mode){
	if(mode == GPS::MODE::GLL_MODE)
	{
		UART->SendString(GPS::AlwaysLocateMode);
		UART->SendString(GPS::GLLMode);
	}
	//else ...

}

void GPS::SetOrigin(float Lat, float Long){
	Latitude_Origin = Lat;
	Longitude_Origin = Long;
}

void GPS::SetOrigin(){
	Latitude_Origin = Latitude;
	Longitude_Origin = Longitude;
}

double GPS::GetX(){
	double deltaLong = Longitude - Longitude_Origin;
	X = a*deltaLong*cos(Latitude); //x+ LESTE
//	X = a*cos(deltalat) * sin(deltalong)
	return X;
}

double GPS::GetY(){
	double deltaLat = Latitude - Latitude_Origin;
	Y = a*deltaLat;
	//	Y = a*cos(deltalat) * cos(deltalong)
	return Y;
}

double GPS::GetLatitude(){
	return Latitude;
}

double GPS::GetLongitude(){
	return Longitude;
}


bool GPS::GetLatLong(){
	char* buffer = (char*)UART->GetBuffer();
	char* ptr;

    double lati;
    double longi;

    double lati_d;
    double longi_d;
    double lati_m;
    double longi_m;
	ptr = strchr(buffer,','); // look for comma

			lati = atof(++ptr);
			if(lati == 0) //testa dado valido (latitude = 0 nao existe)
				return false; //filtra logo
//
			ptr = strchr((char*)ptr,','); //procura 2a virgula ddmm.mmmm
			ptr = strchr((char*)++ptr,','); //procura 3a virgula dddmm.mmmm
			longi = atof(++ptr);
			lati_d = floorf(lati/100); //graus
			lati_m = (lati/100.0 - lati_d)*1.66666666666666; //isola os minutos, converte para décimos de graus (1 min = 1/60 grau)
//			//exemplo : min = 44.6567. 44.6567/60 = 0.744
			longi_d = floorf(longi/100); //graus
			longi_m = (longi/100.0 - longi_d)*1.6666666666666;//isola os minutos, converte para décimos de graus
			lati = lati_d + lati_m;
			longi = longi_d + longi_m;
			Latitude = deg2rad(-lati);
			Longitude = deg2rad(-longi);

//			GPSData->latitude = deg2rad(-lati);
//			GPSData->longitude = deg2rad(-longi);
			UART->ClearBuffer();
	return true;
}

bool GPS::WaitData(){
	return UART->GetTerminator();
}

