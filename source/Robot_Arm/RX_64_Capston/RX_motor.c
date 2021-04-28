
/*
 * RX_motor.c
 *
 * Created: 2020-08-29 오후 10:12:45
 *  Author: LEEHUIDO
 */ 
#define F_CPU 16000000UL
//#define RS485_TXD PORTE &= ~_BV(PE3),PORTE |= _BV(PE2)    		//_485_DIRECTION = 1

DDRF = 0b00000001;
//#define RS485_TXD	PORTF = 0b00000001;
//#define RS485_RXD	PORTF = 0b00000000;
#define RS485_TXD	PORTD |= 0b00010000;
#define RS485_RXD	PORTD &= 0b11101111;

#define sbi(REG8,BITNUM) REG8 |= _BV(BITNUM)

#include "RX_motor.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

byte gbpTxBuffer[128];
byte gbpParameter[128];

void uart_mx_init(unsigned int ubrr)
{
	
	UBRR1H = (unsigned char)(ubrr>>8);    // Set baud rate
	UBRR1L = (unsigned char)ubrr;
	
	UCSR1A = 1<<U2X1;						// 비동기 2배속 모드 , U2X1==1
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);		// Enable receiver and transmitter(송,수신 가능하게 함) , RXEN1==4  , TXEN1==3
	UCSR1C = 3<<UCSZ10;					// Set frame format: 8data, 1stop bit    ,   UCSZ10==1
}

void uart_mx_transmit(unsigned char data)
{
	while(!(UCSR1A & (1<<UDRE1)));		//UDRE1 = 5  , 레지스터 UCSR1A의 5비트는 자동(전송준비가 되면 1이된다)

	UDR1=data;
}

byte TxPacket_mx (byte bID,byte blnstruction,byte bParameterLength)	//ID값,Instruction,parameter 길이
{
	byte bCount,bCheckSum,bPacketLength;

	gbpTxBuffer[0] = 0xff;				//패킷의 시작을 알리는 신호
	gbpTxBuffer[1] = 0xff;				//패킷의 시작을 알리는 신호
	gbpTxBuffer[2] = bID;				//ID
	gbpTxBuffer[3] = bParameterLength+2;	//패킷의 길이  Instruction(1) + Parameter0(1) + ParameterN(N) Parameter 개수(N) + 2

	
	gbpTxBuffer[4] = blnstruction;		//Dynamixel에게 수행하라고 지시하는 명령.

	
	for(bCount = 0; bCount < bParameterLength; bCount++)		//Put gbpParameter Value in gbpTxBuffer
	{
		gbpTxBuffer[bCount+5] = gbpParameter[bCount];
	}

	bCheckSum = 0;
	bPacketLength = bParameterLength+6;			//StartByte(2) + IDByte(1) + LengthByte(1) + Instruction(1) + Parameter(1+N)

	for(bCount = 2; bCount<bPacketLength-1; bCount++)	//StartByte 제외, Parameter0 포함, CheckSum 계산
	{
		bCheckSum += gbpTxBuffer[bCount];
	}

	
	gbpTxBuffer[bCount] = ~bCheckSum;		// Check Sum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )


	RS485_TXD
	_delay_ms(1);
	for(bCount=0;bCount<bPacketLength;bCount++)	//uart통신 Packet 전송
	{
		sbi(UCSR1A,6);
		uart_mx_transmit(gbpTxBuffer[bCount]);
	}
	while(!CHECK_TXD1_FINISH);		//전송이 끝날때 까지 대기
	RS485_RXD
	_delay_ms(1);	
	return(bPacketLength);		//Packet길이 반환
}

byte mx_ID(unsigned int ID_number)		//ID설정 함수
{
	gbpParameter[0] = 0x03;				//ID address
	gbpParameter[1] = ID_number;
	
	TxPacket_mx (0xFE,0x03,2);
}

byte mx_LED(unsigned int ID_number, unsigned int state)
{
	gbpParameter[0] = 0x19;				//ID address
	gbpParameter[1] = state;
	
	TxPacket_mx (ID_number,0x03,2);
}

byte mx_position(unsigned int p_number,unsigned int ID_number)			//모터 위치값,모터 ID값
{
	unsigned int position = 3.41*p_number;	//Change 0~300 to 0~1023
	gbpParameter[0] = 0x1E; //goal position(L) address
	gbpParameter[1] = (unsigned char)(position); //Writing Data  , goal position(L)
	gbpParameter[2] = (unsigned char)(position>>8); //goal position(H)

	TxPacket_mx (ID_number,0x03,3);	// , 0x03명령, 길이
}


void borate_set(unsigned int borate)		// 0x01 == 1M , 0x03 == 500000 , 0x04 == 400000 , 0x07 == 250000 , 0x09 == 200000  보레이트 변경하는 함수
{
	gbpParameter[0] = 0x04;		//Baud Rate Address
	gbpParameter[1] = borate; // 1M
	
	TxPacket_mx (0xFE,0x03,2); //모터 전체 , 0X03명령 , 길이
}


void All_Control_TEST(	unsigned int ID_number_1, int chose_1, unsigned int flexible_val_1,unsigned int flexible_1, unsigned int p_number_1, unsigned int AX_Speed_1,
unsigned int ID_number_2, int chose_2, unsigned int flexible_val_2,unsigned int flexible_2, unsigned int p_number_2, unsigned int AX_Speed_2,
unsigned int ID_number_3, int chose_3, unsigned int flexible_val_3,unsigned int flexible_3, unsigned int p_number_3, unsigned int AX_Speed_3,
unsigned int ID_number_4, int chose_4, unsigned int flexible_val_4,unsigned int flexible_4, unsigned int p_number_4, unsigned int AX_Speed_4,
unsigned int ID_number_5, int chose_5, unsigned int flexible_val_5,unsigned int flexible_5, unsigned int p_number_5, unsigned int AX_Speed_5)
{
	unsigned int position_1 = 3.41*p_number_1;			//Change 0~300 to 0~1023

	unsigned int position_2 = 3.41*p_number_2;			//Change 0~300 to 0~1023
	unsigned int position_2a = 3.41*(300-p_number_2);	//Change 0~300 to 0~1023

	unsigned int position_3 = 3.41*p_number_3;			//Change 0~300 to 0~1023
	unsigned int position_4 = 3.41*p_number_4;			//Change 0~300 to 0~1023
	unsigned int position_5 = 3.41*p_number_5;			//Change 0~300 to 0~1023

	unsigned int AX_Speed_5_c = 9*AX_Speed_5;
	unsigned int AX_Speed_4_c = 9*AX_Speed_4;
	unsigned int AX_Speed_3_c = 9*AX_Speed_3;
	unsigned int AX_Speed_2_c = 9*AX_Speed_2;
	unsigned int AX_Speed_1_c = 9*AX_Speed_1;
	
	int CheckSum_Value= 0;
	byte bCount,bCheckSum,bPacketLength;

	gbpTxBuffer[0] = 0xFF;
	gbpTxBuffer[1] = 0xFF;	//시작
	gbpTxBuffer[2] = 0xFE;	//ID

	gbpTxBuffer[3] = ((8 + 1)*5) + 4;	//길이 (L:Dynamixel별 Data Length, N:Dynamixel의 개수)

	gbpTxBuffer[4] = 0x83;	//명령

	gbpTxBuffer[5] =0x1a;
	gbpTxBuffer[6] =0x08;

	gbpTxBuffer[7] = 0x01;				//ID
	gbpTxBuffer[8] = flexible_val_1;	//유격 오차
	gbpTxBuffer[9] = flexible_val_1;	//유격 오차
	gbpTxBuffer[10] = flexible_1;		//유연성
	gbpTxBuffer[11] = flexible_1;		//유연성
	gbpTxBuffer[12] = (unsigned char)(position_1); //Writing Data  , goal position(L)
	gbpTxBuffer[13] = (unsigned char)(position_1>>8); //goal position(H)
	gbpTxBuffer[14] = (unsigned char)AX_Speed_1_c; //goal Speed(L)
	gbpTxBuffer[15] = (unsigned char)(AX_Speed_1_c >> 8); //goal Speed(H)

	gbpTxBuffer[16] = 0x02;				//ID
	gbpTxBuffer[17] = flexible_val_2;	//유격 오차
	gbpTxBuffer[18] = flexible_val_2;	//유격 오차
	gbpTxBuffer[19] = flexible_2;		//유연성
	gbpTxBuffer[20] = flexible_2;		//유연성
	gbpTxBuffer[21] = (unsigned char)(position_2); //Writing Data  , goal position(L)
	gbpTxBuffer[22] = (unsigned char)(position_2>>8); //goal position(H)
	gbpTxBuffer[23] = (unsigned char)AX_Speed_2_c; //goal Speed(L)
	gbpTxBuffer[24] = (unsigned char)(AX_Speed_2_c >> 8); //goal Speed(H)

	gbpTxBuffer[25] = 0x03;				//ID
	gbpTxBuffer[26] = flexible_val_2;	//유격 오차
	gbpTxBuffer[27] = flexible_val_2;	//유격 오차
	gbpTxBuffer[28] = flexible_2;		//유연성
	gbpTxBuffer[29] = flexible_2;		//유연성
	gbpTxBuffer[30] = (unsigned char)(position_2a); //Writing Data  , goal position(L)
	gbpTxBuffer[31] = (unsigned char)(position_2a>>8); //goal position(H)
	gbpTxBuffer[32] = (unsigned char)AX_Speed_2_c; //goal Speed(L)
	gbpTxBuffer[33] = (unsigned char)(AX_Speed_2_c >> 8); //goal Speed(H)
	
	gbpTxBuffer[34] = 0x04;				//ID
	gbpTxBuffer[35] = flexible_val_4;	//유격 오차
	gbpTxBuffer[36] = flexible_val_4;	//유격 오차
	gbpTxBuffer[37] = flexible_4;		//유연성
	gbpTxBuffer[38] = flexible_4;		//유연성
	gbpTxBuffer[39] = (unsigned char)(position_4); //Writing Data  , goal position(L)
	gbpTxBuffer[40] = (unsigned char)(position_4>>8); //goal position(H)
	gbpTxBuffer[41] = (unsigned char)AX_Speed_4_c; //goal Speed(L)
	gbpTxBuffer[42] = (unsigned char)(AX_Speed_4_c >> 8); //goal Speed(H)
	
	gbpTxBuffer[43] = 0x05;				//ID
	gbpTxBuffer[44] = flexible_val_5;	//유격 오차
	gbpTxBuffer[45] = flexible_val_5;	//유격 오차
	gbpTxBuffer[46] = flexible_5;		//유연성
	gbpTxBuffer[47] = flexible_5;		//유연성
	gbpTxBuffer[48] = (unsigned char)(position_5); //Writing Data  , goal position(L)
	gbpTxBuffer[49] = (unsigned char)(position_5>>8); //goal position(H)
	gbpTxBuffer[50] = (unsigned char)AX_Speed_5_c; //goal Speed(L)
	gbpTxBuffer[51] = (unsigned char)(AX_Speed_5_c >> 8); //goal Speed(H)

	for(int i=2; i<=51; i++)
	{
		CheckSum_Value+= gbpTxBuffer[i];
	}
	gbpTxBuffer[52] = ~(CheckSum_Value);	//Check Sum
	
	RS485_TXD
	_delay_ms(1);	
	bPacketLength= 0x35;
	for(bCount=0;bCount<bPacketLength;bCount++)	//uart통신 Packet 전송
	{
		sbi(UCSR1A,6);
		uart_mx_transmit(gbpTxBuffer[bCount]);
	}
	while(!CHECK_TXD1_FINISH);		//전송이 끝날때 까지 대기
	RS485_RXD
	_delay_ms(1);
	return(bPacketLength);		//Packet길이 반환
}