/*
 * RX_motor.h
 *
 * Created: 2020-08-29 오후 10:12:56
 *  Author: LEEHUIDO
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

typedef unsigned char byte;
#define CHECK_TXD1_FINISH bit_is_set(UCSR1A,6)				//UCSR1A의 6비트는 송신을 다했으면 1


void uart_mx_init(unsigned int ubrr);
void uart_mx_transmit(unsigned char data);
byte mx_ID(unsigned int ID_number);
byte mx_position(unsigned int p_number,unsigned int ID_number);			//모터 위치값,모터 ID값
byte mx_LED(unsigned int ID_number, unsigned int state);
byte TxPacket_mx (byte bID,byte blnstruction,byte bParameterLength);	//ID값,Instruction,parameter 길이
void borate_set(unsigned int borate);
byte Factory_Reset(unsigned int ID_number);		//ID설정 함수


