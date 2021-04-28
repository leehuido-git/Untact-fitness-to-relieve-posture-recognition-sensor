/*
 * UART.c
 *
 * Created: 2020-08-29 오후 10:15:27
 *  Author: LEEHUIDO
 */ 
#include "UART.h"
unsigned char uart0_rx_flag =0;
unsigned char uart0_rx_data;

byte gbpTxBuffer[128];
byte gbpParameter[128];

#define sbi(REG8,BITNUM) REG8 |= _BV(BITNUM)
static FILE mystdout = FDEV_SETUP_STREAM(UART_putchar, NULL, _FDEV_SETUP_WRITE);
// FDEV_SETUP_STREAM은 매크로 함수이다. 즉 매크로 함수를 통해서 위 소스 코드에서 정의한 UART_putchar함수와 printf를 연결해주고 있다.

ISR (USART0_RX_vect)		// 어떤 인터럽트가 발생했을 때 이를 받아서 처리하는 함수를 ISR함수라고 한다.
{
	uart0_rx_data= UDR0;	//UDR0 :	USART0 포트의 송수신 데이터 버퍼의 기능을 수행한다.
	uart0_rx_flag= 1;
}

void UART0_init(long baud)
{
	unsigned short ubrr = (F_CPU/ (8 * baud)-1);	//보레이트를 결정하는 공식 (비동기 2배속 모드)
	UBRR0H= (unsigned char)(ubrr >> 8);
	UBRR0L= (unsigned char)ubrr;
	
	UCSR0A= (1 << U2X0);	//비동기 2배속 모드
	//	UCSR0A 레지스터는 USART0 포트의 송수신 동작을 송수신 상태를 저장하는 기능을 수행한다.

	UCSR0B= (1 << RXCIE0) | (0 << TXCIE0) | (1<< RXEN0) | (1 << TXEN0);
	//	UCSROB 레지스터는 USART0 포트의 송수신 동작을 제어
	//	RXCIE0, TXCIE0: 수신완료,송신완료 인터럽트를 개별적으로 허용하는 비트, RXEN0, TXEN0: 송신 데이터, 수신 데이터 레지스터 준비완료 인터럽트를 개별적으로 허용하는 비트
	UCSR0C= (1 << UCSZ01) | (1 << UCSZ00);
	//	UCSR0C 레지스터는 USART0 포트의 송수신 동작을 제어하는 기능
	//	UCSZ01: 전송 문자의 데이터 비트수를 설정 (8비트로 설정됨)
	stdout= &mystdout;
	//stdout: 표준 출력
}

void UART0_putchar(char c)
{
	if(c == '\n') UART0_putchar('\r');
	while(!(UCSR0A &(1 << UDRE0)));
	UDR0 =c;
}

void UART_putchar(char c, FILE * stream)
{
	if(c == '\n') UART_putchar('\r', stream);
	while(!(UCSR0A & (1<< UDRE0)));
	UDR0 =c;
}

void UART0_puts(char *c)
{
	int i=0;
	while(1)
	{
		if(c[i] == NULL)
		{
			break;
		}
		else
		{
			UART0_putchar(c[i]);
			i++;
		}
	}
}

int UART0_getchar(char *c)
{
	if (uart0_rx_flag == 0)
	{
		return 0;
	}
	else
	{
		*c = uart0_rx_data;
		uart0_rx_flag = 0;
		return 1;
	}
}
#include <string.h>
/*
int UART0_gets(char *s)
{
	char data;
	if (UART0_getchar(&data) == 1)
	{
		if (data == '\n')
		{
			*s++ = data;
			return 1;
		}
		else
		{
			*s++ = data;
		}
	}
	else
	{
		return 0;
	}
}
*/
int UART0_gets(char *s, int n)
{
	char *c;
	int i = 0;
	while(1)
	{
		if(UART0_getchar(c))
		{
			s[i] = *c;
			if(s[i] == '\n')
			{
				if(s[0] == 's' && s[4] == ',' && s[8] == ',' && s[12] == ',' && s[16] == ',' && s[20] == ',')
				{
					return 1;
				}
				else
				{
					return 0;
				}
			}
			i++;			
		}
	}
}
/*
char UART_getc(void)
{
	// wait for data
	while(!(UCSR0A & (1 << RXC0)));

	// return data
//	return (int)UDR0;
	return UDR0;
}
*/
/*
void UART_getLine(char* buf, uint8_t n)
{
	uint8_t bufIdx = 2;
	uint8_t check = 0;
	char c;
	do
	{
		while(!(check == 2))
		{
			c = UART_getc();
			if(c == 0xFE)
			{
				check++;
			}
		}
		c = UART_getc();

		// store character in buffer
		buf[bufIdx++] = c;
	}
	while((bufIdx < n));

	// ensure buffer is null terminated
	buf[0] = 0xFE;
	buf[1] = 0xFE;	
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////
/*
void uart_transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));		//UDRE1 = 5  , 레지스터 UCSR1A의 5비트는 자동(전송준비가 되면 1이된다)

	UDR0=data;
}

byte TxPacket (byte bID,byte blnstruction,byte bParameterLength)	//ID값,Instruction,parameter 길이
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


	for(bCount=0;bCount<bPacketLength;bCount++)	//uart통신 Packet 전송
	{
		sbi(UCSR0A,6);
		uart_transmit(gbpTxBuffer[bCount]);
	}
	while(!CHECK_TXD0_FINISH);		//전송이 끝날때 까지 대기
	return(bPacketLength);		//Packet길이 반환
}

void TEST(void)
{
	gbpParameter[0] = 0x03;		//Baud Rate Address
	gbpParameter[1] = 0x02; // 1M
	
	TxPacket (0xFE,0x03,2); //모터 전체 , 0X03명령 , 길이
}
*/