/* Projektni zadatak iz Autoelektronike - "Simulacija LIN komunikacije" + Misra
   Studenti: Milomir Spajic EE83/2017
			 Djordje Djozlija EE50/2017
   septembar 2021.  */

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL TO USE */
#define COM_CH_0 (0)
#define COM_CH_1 (1)

/* TASK PRIORITIES */

/* TASKS: FORWARD DECLARATIONS */

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* GLOBAL OS-HANDLES */


/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	

    vTaskStartScheduler();

}