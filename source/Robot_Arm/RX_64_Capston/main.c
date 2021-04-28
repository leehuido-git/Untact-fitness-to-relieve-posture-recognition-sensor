/*
 * RX_64_Capston.c
 *
 * Created: 2020-08-29 오후 10:08:16
 * Author : LEEHUIDO
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "RX_motor.h"
#include "UART.h"
#include "Interrupt.h"
#define ON 1
#define OFF 0
int start = 0;
int INIT = 0;

ISR (INT4_vect)
{
	INIT = 1;
	_delay_ms(100);
}

int main(void)
{
	char* c;
	DDRD = 0xff;
//	DDRA = 0xff;
	sei();
	Interrupt_init();
	UART0_init(115200);
	uart_mx_init(1);
	DDRE = 0b00000011;
	DDRA = 0b00001111;
//	borate_set(1);
//	mx_ID(0x05);
	int yaw[3];
	int pitch[3];
	int roll[3];	

	int yaw_init[3];
	int pitch_init[3];
	int roll_init[3];	
	uint8_t bufSize = 24;
	char buf[bufSize];
	_delay_ms(2000);

//	mx_position(180, 0x05);
//	_delay_ms(2000);
//	mx_position(130, 0x05);	
//	_delay_ms(2000);
//	mx_position(150, 0x05);
	All_Control_TEST(
	0x01, ON, 1, 32,	150,	50,
	0x02, ON, 1, 32,	150,	50,
	0x03, ON, 1, 32,	150,	50,
	0x04, ON, 1, 32,	150,	50,
	0x05, ON, 1, 32,	150,	50);
	_delay_ms(2000);
    while (1)
    {
		if(UART0_gets(buf, bufSize))
		{
			yaw[1] = ((int)(buf[1] - '0')*100 + (int)(buf[2] - '0')*10 + (int)(buf[3] - '0')) - 500;
			yaw[2] = ((int)(buf[13] - '0')*100 + (int)(buf[14] - '0')*10 + (int)(buf[15] - '0')) - 500;
			pitch[1] = ((int)(buf[5] - '0')*100 + (int)(buf[6] - '0')*10 + (int)(buf[7] - '0')) - 500;
			pitch[2] = ((int)(buf[17] - '0')*100 + (int)(buf[18] - '0')*10 + (int)(buf[19] - '0')) - 500;
			roll[1] = ((int)(buf[9] - '0')*100 + (int)(buf[10] - '0')*10 + (int)(buf[11] - '0')) - 500;
			roll[2] = ((int)(buf[21] - '0')*100 + (int)(buf[22] - '0')*10 + (int)(buf[23] - '0')) - 500;			
			if(INIT)
			{
				for(int i = 0; i < 3; i++)
				{
					yaw_init[i] = yaw[i];
					pitch_init[i] = pitch[i];
					roll_init[i] = roll[i];					
				}
				INIT = 0;
				start = 1;
			}
			if(start)
			{
				for(int i = 1; i < 3; i++)
				{
					yaw[i] = yaw[i] - yaw_init[i] + 150;
					pitch[i] = pitch[i] - pitch_init[i] + 150;
					roll[i] = roll[i] - roll_init[i] + 150;
				}
				printf("Yaw_1 = %d, Pitch_1 = %d, Roll_1 = %d, Yaw_2 = %d, Pitch_2 = %d, Roll_2 = %d\n", -yaw[1], -pitch[1], roll[1], (300 - yaw[2]), (300 - pitch[2]), roll[2]);				
			All_Control_TEST(
			0x01, ON, 1, 32,	(300 - yaw[2]),	50,
			0x02, ON, 1, 32,	(300 - pitch[2]),	50,
			0x03, ON, 1, 32,	150,	50,
			0x04, ON, 1, 32,	150,	50,
			0x05, ON, 1, 32,	150,	50);				
			}
			else
			{
				//값 확인용
				printf("Yaw_1 = %d, Pitch_1 = %d, Roll_1 = %d, Yaw_2 = %d, Pitch_2 = %d, Roll_2 = %d\n", yaw[1], pitch[1], roll[1], yaw[2], pitch[2], roll[2]);				
			}
		}		
/*
*/
		// get line from UART
/*		
		UART_getLine(buf, bufSize);
		byte checksum = (byte)~(buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]+buf[11]+buf[12]+buf[13]);
		printf("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", buf[0],buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14]);
		if(!checksum)
		{
			printf("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n", buf[0],buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[13]);
			printf("%2x\n", checksum);
			
			yaw = (((((int)buf[3]*100) + (int)buf[4]) - 500));
			pitch = (((((int)buf[5]*100) + (int)buf[6]) - 500 +150));
			roll =  (((((int)buf[7]*100) + (int)buf[8]) - 500));
			if(yaw >=300)
			{
				yaw = 300;
			}
//			mx_position(yaw, 0xFE);
			All_Control_TEST(
			0x01, ON, 1, 32,	yaw,	50,
			0x02, ON, 10, 32,	pitch,	50,
			0x03, ON, 1, 32,	150,	50,
			0x04, ON, 1, 32,	150,	50,
			0x05, ON, 1, 32,	150,	50);
			printf("Yaw: %d, Pitch: %d, Roll: %d\n", yaw, pitch, roll);			
		}
		else
		{
			printf("error\n");
		}	
*/		
	}
}