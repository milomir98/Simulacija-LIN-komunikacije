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
#define SERVICE_TASK_PRI (tskIDLE_PRIORITY + (UBaseType_t)1)

/* TASKS: FORWARD DECLARATIONS */
static void led_bar_tsk(void* pvParameters);

void main_demo(void);

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* GLOBAL OS-HANDLES */

// Task koji na osnovu pritisnutog tastera na led baru ispisuje na 7seg displej informaciju, brzina osvjezavanja je 100ms

static void led_bar_tsk(void* pvParameters)
{
	uint8_t senzor1, senzor2, senzor3, senzor4, senzor5;

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));

		// Provjera koji taster sa LED bara je pritisnut //
		if (get_LED_BAR(1, &senzor1) != 0)
		{
			printf("Greska prilikom ocitavanja senzora");
		}
		if (get_LED_BAR(2, &senzor2) != 0)
		{
			printf("Greska prilikom ocitavanja senzora");
		}
		if (get_LED_BAR(3, &senzor3) != 0)
		{
			printf("Greska prilikom ocitavanja senzora");
		}
		if (get_LED_BAR(4, &senzor4) != 0)
		{
			printf("Greska prilikom ocitavanja senzora");
		}
		if (get_LED_BAR(5, &senzor5) != 0)
		{
			printf("Greska prilikom ocitavanja senzora");
		}

		// Ispis stanja senzora 1 na 7seg displej //
		if (senzor1 == (uint8_t)1)
		{
			if (select_7seg_digit(0) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[1]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
		else
		{
			if (select_7seg_digit(0) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[0]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}

		// Ispis stanja senzora 2 na 7seg displej //
		if (senzor2 == (uint8_t)1)
		{
			if (select_7seg_digit(1) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[2]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
		else
		{
			if (select_7seg_digit(1) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[0]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}

		// Ispis stanja senzora 3 na 7seg displej //
		if (senzor3 == (uint8_t)1)
		{
			if (select_7seg_digit(2) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[3]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
		else
		{
			if (select_7seg_digit(2) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[0]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}

		// Ispis stanja senzora 4 na 7seg displej //
		if (senzor4 == (uint8_t)1)
		{
			if (select_7seg_digit(3) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[4]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
		else
		{
			if (select_7seg_digit(3) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[0]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}

		// Ispis stanja senzora 5 na 7seg displej //
		if (senzor5 == (uint8_t)1)
		{
			if (select_7seg_digit(4) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[5]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
		else
		{
			if (select_7seg_digit(4) != 0)
			{
				printf("Greska prilikom izbora cifre 7seg displeja");
			}
			if (set_7seg_digit(hexnum[0]) != 0)
			{
				printf("Greska prilikom ispisa cifre na 7seg displej");
			}
		}
	}
}

/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	// Inicijalizacija periferija //
	if (init_LED_comm() != 0)
	{
		printf("Neuspjesna inicijalizacija");
	}
	if (init_7seg_comm() != 0)
	{
		printf("Neupsjesna inicijalizacija");
	}



	// Kreiranje taskova //
	BaseType_t status;
	status = xTaskCreate(
		led_bar_tsk,
		"led_bar_tsk",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	

    vTaskStartScheduler();

}