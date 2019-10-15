/*
 * Uart.cpp
 *
 *  Created on: 16 de mai de 2019
 *      Author: marcus
 */

#include "Uart.h"

// TODO Auto-generated constructor stub
Uart::Uart(UART_Type* Per) {

	Peripheral = Per;
	buffer_index = 0;
	Terminator = false;
	memset(Buffer,0,UART_BUFFER_SIZE);

}


void Uart::Send(const uint8_t* data,uint8_t n){
	UART_WriteBlocking(Peripheral, data, n);
}

void Uart::SendString(const uint8_t* string){
	uint8_t len = strlen((const char*)string);
	UART_WriteBlocking(Peripheral, string, len);
}

void Uart::InterruptHandle(const uint8_t byte){
	Buffer[buffer_index++] = byte;
	if(byte == UART_TERMINATOR){
		Terminator = true;
		Buffer[buffer_index] = 0;
		buffer_index = 0;
	}
	if(buffer_index == UART_BUFFER_SIZE)
		buffer_index = 0;

}

bool Uart::GetTerminator(){
	if(Terminator == true){
	Terminator = false;
	return true;
	}
	return false;
}

uint8_t* Uart::GetBuffer(){
	return Buffer;
}


void Uart::ClearBuffer(){
	memset(Buffer,0,UART_BUFFER_SIZE);
	buffer_index = 0;
}

