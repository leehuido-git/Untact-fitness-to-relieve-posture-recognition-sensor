/*
 * Interrupt.c
 *
 * Created: 2020-09-28 오후 1:58:39
 *  Author: LEEHUIDO
 */ 
#include "Interrupt.h"
#include <avr/io.h>
#include <avr/interrupt.h>

void Interrupt_init(void)
{
	EICRB = (1 << ISC41) | (0 << ISC41);
	EIMSK = (1 << INT4);
}