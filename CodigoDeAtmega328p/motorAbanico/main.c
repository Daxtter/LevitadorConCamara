/*
 * PWM_UART_CONTROL_16.c
 *
 * Created: 9/24/2024 8:58:52 AM
 * Author : karoe
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include "UART.h"

void configurePins();

bool rxflag=false;
int count=0;
uint16_t dato=0,datoAnt=0;
uint8_t datoLow,datoHigh;

int main(void)
{
	
	UART_init();
	configurePins();
    /* Replace with your application code */
    while (1) 
    {
		if(rxflag)
		{
			dato = (datoHigh << 8) | datoLow;
			rxflag = false;
		}
		
		if(dato!=datoAnt)
		{
			OCR1A=dato;
			datoAnt=dato;
		}
    }
}

void configurePins()
{
	DDRB |= 1<<PINB1; //B1 Output
	TCCR1A |= (1<<COM1A1)|(0<<COM1A0); //Timer1 Config. PWM (Non Inverting)
	TCCR1A |= (0<<WGM11)|(0<<WGM10); //fast PWM 
	TCCR1B |= (1<<WGM13)|(0<<WGM12); //TOP=ICR1
	TCCR1B |= (0<<CS12)|(0<<CS11)|(1<<CS10); // CLK NO prescaler
	ICR1=0xFFFF;
	OCR1A=0;
	sei(); //enable global interrupts
}

ISR(USART_RX_vect)
{
	count++;
	if(count==1) datoLow=UART_read();
	if(count==2)
	{
		datoHigh=UART_read();
		rxflag=true;
		count=0;
	}
}