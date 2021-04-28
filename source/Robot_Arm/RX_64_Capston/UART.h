/*
 * UART.h
 *
 * Created: 2020-08-29 오후 10:15:40
 *  Author: LEEHUIDO
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define CHECK_TXD0_FINISH bit_is_set(UCSR0A,6)				//UCSR1A의 6비트는 송신을 다했으면 1

typedef unsigned char byte;

void UART0_init(long baud);
void UART0_putchar(char c);
void UART_putchar(char c, FILE * stream);
void UART0_puts(char *c);
int UART0_getchar(char *c);
int UART0_gets(char *s, int n);
//char UART_getc(void);
//void UART_getLine(char* buf, uint8_t n);

//void uart_transmit(unsigned char data);