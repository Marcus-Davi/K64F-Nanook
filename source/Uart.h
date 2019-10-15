/*
 * Uart.hpp
 *
 *  Created on: 16 de mai de 2019
 *      Author: marcus
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "fsl_uart.h"


#define UART_BUFFER_SIZE 80
#define UART_TERMINATOR '\n'


class Uart {
private:
	UART_Type* Peripheral;
	unsigned char buffer_index;
	uint8_t Buffer[UART_BUFFER_SIZE];
	bool Terminator;

public:

	Uart(UART_Type* Per);

	void Send(const uint8_t* data,uint8_t n);
	void SendString(const uint8_t* string);
	void InterruptHandle(const uint8_t byte);
	bool GetTerminator();
	uint8_t* GetBuffer();
	void ClearBuffer();

};

#endif /* INC_UART_H_ */
