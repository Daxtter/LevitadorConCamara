#ifndef UART_H_
#define UART_H_
#define PIN PINB
#define F_CPU 16000000
#include <avr/io.h>

void UART_init(); //Initialize:Async/D:8bit/S:1bit/9600B
void UART_init_para(uint8_t,uint8_t,uint8_t,uint8_t ,uint8_t,uint8_t,uint8_t);//init con parametros
unsigned char UART_read();	//Data read function
void UART_write(unsigned char); //Data write function
void UART_string(char*);	//String write function

void UART_init()
{
	
	UCSR0A |= (0<<U2X0); //Double USART Speed Disabled
	UCSR0A |= (0<<MPCM0); //Multiprocessor Mode Disabled
	UCSR0B |= (1<<TXEN0); // Tx Enable
	UCSR0B |= (1<<RXEN0); // Rx Enable
	UCSR0B |= (0<<UDRIE0); // Data Register Empty IntEn
	UCSR0B |= (0<<TXCIE0); // Tx Complete IntEn
	UCSR0B |= (1<<RXCIE0); // Rx Complete IntEn
	UCSR0C |= (0<<UMSEL01)|(0<<UMSEL00); //Mode: 00=Asynchronous
	UCSR0C |= (0<<UPM01)|(0<<UPM00); // Parity Mode: 00=None
	UCSR0C |= (0<<USBS0); // Stop bit: 0=1-bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00); //Data Size: 11=8-bit
	UCSR0C |= (0<<UCPOL0); //Clock Polarity: 0 for Async.
	UBRR0 = 103; //baud rate: 103=9600B a 16MHz
}

void UART_init_para(uint8_t baudRate,uint8_t dataSize,uint8_t parityMode,uint8_t stopBits,uint8_t txInt,uint8_t rxInt,uint8_t DREInt)
{
	UCSR0A=0b00000000;
	UCSR0B=0b00000000; //limpiar registros
	UCSR0C=0b00000000;
	
	UCSR0A |= (0<<U2X0); //Double USART Speed
	UCSR0A |= (0<<MPCM0); //Multiprocessor Mode
	UCSR0B |= (1<<TXEN0); // Tx Enable
	UCSR0B |= (1<<RXEN0); // Rx Enable
	UCSR0C |= (0<<UMSEL01)|(0<<UMSEL00); //Mode
	UCSR0C |= (0<<UCPOL0); //Clock Polarity
	
	if(DREInt==1) UCSR0B |= (1<<UDRIE0); // Data Register Empty Int
	else UCSR0B &= ~(1<<UDRIE0);
	
	if(rxInt==1) UCSR0B |= (1<<TXCIE0); // Tx Complete Int
	else UCSR0B &= ~(1<<TXCIE0);
	
	if(txInt==1) UCSR0B |= (1<<RXCIE0); // Rx Complete Int
	else UCSR0B &= ~(1<<RXCIE0);
	
	if(stopBits==1) UCSR0C |= (1<<USBS0); // Stop bit  =1bit
	else  UCSR0C &= ~(1<<USBS0); //=2bit
	
	switch(parityMode)// Parity Modes
	{
		case 0b00000001:
		UCSR0C |= (0<<UPM01)|(1<<UPM00); //synchronous
		break;
		case 0b00000011:
		UCSR0C |= (1<<UPM01)|(1<<UPM00); //master SPI
		break;
		default:
		UCSR0C |= (0<<UPM01)|(0<<UPM00); //asynchronous
	}
	
	switch(dataSize)// Data Size
	{
		case 0b00000001: //6bit
		UCSR0C |= (0<<UCSZ01)|(1<<UCSZ00);
		UCSR0B |= (0<<UCSZ02);
		break;
		case 0b00000010: //7bit
		UCSR0C |= (1<<UCSZ01)|(0<<UCSZ00);
		UCSR0B |= (0<<UCSZ02);
		break;
		case 0b00000011: //8bit
		UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
		UCSR0B |= (0<<UCSZ02);
		break;
		case 0b00000111: //9bit
		UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
		UCSR0B |= (1<<UCSZ02);
		break;
		default:	//5bit
		UCSR0C |= (0<<UCSZ01)|(0<<UCSZ00);
		UCSR0B |= (0<<UCSZ02);
	}
	
	UBRR0 = baudRate; //baud rate
}

unsigned char UART_read()
{
	if(UCSR0A&(1<<RXC0))// If Rx Complete Flag in UCSR0A is Set
	{
		return UDR0;// Return received data in UDR0
	}
	return 0;//Else Return 0
}

void UART_write(unsigned char data)
{
	while(!(UCSR0A&(1<<UDRE0)));// While Data Register is not Empty
	UDR0 = data;// Put data into buffer, sends the data
}

void UART_string(char* string)
{
	while(*string != 0)// While the last value is not null
	{
		UART_write(*string);// Send the next character in the string
		string++;// Add one to the string address
	}
}

#endif /* UART.H */