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
//#define COM_CH_0 (0)
#define COM_CH_1 (1)
#define COM_CH_2 (2)
#define COM_CH_3 (3)
#define COM_CH_4 (4)

/* TASK PRIORITIES */
#define SERVICE_TASK_PRI		(tskIDLE_PRIORITY + (UBaseType_t)1)
#define TASK_SERIAL_SEND_PRI	(tskIDLE_PRIORITY + (UBaseType_t)2)
#define TASK_SERIAL_REC_PRI		(tskIDLE_PRIORITY + (UBaseType_t)3)

/* TASKS: FORWARD DECLARATIONS */
static void led_bar_tsk(void* pvParameters);
static void slanjeNaPC_tsk(void* pvParameters);
static void parsiranje_tsk(void* pvParameters);
static void ispisAdreseNa7seg_tsk(void* pvParameters);
static void ispisVrijednostiSenzoraNa7seg_tsk(void* pvParameters);
//static void SerialSend_Task0(void* pvParameters);
//static void SerialReceive_Task0(void* pvParameters);
static void SerialSend_Task1(void* pvParameters);
static void SerialReceive_Task1(void* pvParameters);
static void SerialSend_Task2(void* pvParameters);
static void SerialReceive_Task2(void* pvParameters);
static void SerialSend_Task3(void* pvParameters);
static void SerialReceive_Task3(void* pvParameters);
static void SerialSend_Task4(void* pvParameters);
static void SerialReceive_Task4(void* pvParameters);
void main_demo(void);

// GLOBAL VARIABLES //
static char volatile podaciZaPC[16];
static uint8_t t_point;

// RECEPTION DATA BUFFER //
#define R_BUF_SIZE	(32)
static char recBuffer_0[R_BUF_SIZE];
static char recBuffer_1[R_BUF_SIZE];
static char recBuffer_2[R_BUF_SIZE];
static char recBuffer_3[R_BUF_SIZE];
static char recBuffer_4[R_BUF_SIZE];
static uint8_t volatile recPoint_0;
static uint8_t volatile recPoint_1;
static uint8_t volatile recPoint_2;
static uint8_t volatile recPoint_3;
static uint8_t volatile recPoint_4;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t RXC_BinarySemaphore0;
static SemaphoreHandle_t RXC_BinarySemaphore1;
static SemaphoreHandle_t RXC_BinarySemaphore2;
static SemaphoreHandle_t RXC_BinarySemaphore3;
static SemaphoreHandle_t RXC_BinarySemaphore4;
static SemaphoreHandle_t SensID_7_BinSemaphore;
static SemaphoreHandle_t SensID_8_BinSemaphore;
static SemaphoreHandle_t SensID_9_BinSemaphore;
static SemaphoreHandle_t SendToPC_BinSemaphore;
static SemaphoreHandle_t ResponseToPC_BinSemaphore;
//static QueueHandle_t SensorAkumulatorVrijednost_q;
static QueueHandle_t SensorAddress_q;
static QueueHandle_t SensorAddress_7seg_q;
static QueueHandle_t SensorVrijednost_q;
static QueueHandle_t SensorVrijednost_PC_q;
static QueueHandle_t SensorVrijednost_7seg_q;

// Task koji na osnovu pritisnutog tastera na led baru salje adresu odredjenom senzoru
// Skroz donji taster - adresa senzora 0x07, srednji taster - adresa senzora 0x08 i treci taster - adresa senzora 0x09
// Poruke koje se salju su u obliku LIN poruka odnosno frame - ova, tako da se npr. za adresu 0x07 salje poruka oblika 005507
// Poruka se salje u dva reda, jedan se koristi za ispis adrese na 7seg displej a drugi za slanje u taskove koji simuliraju vrijednosti senzora
static void led_bar_tsk(void* pvParameters)
{
	uint8_t senzorOcitavanje;
	char adresaSenzora[7];
	adresaSenzora[0] = '0';
	adresaSenzora[1] = '0';
	adresaSenzora[2] = '5';
	adresaSenzora[3] = '5';
	adresaSenzora[4] = '0';
	adresaSenzora[6] = '\0';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
		
		// Provjera koji taster sa LED bara je pritisnut //
		if (get_LED_BAR(1, &senzorOcitavanje) != 0)
		{
			printf("Greska prilikom ocitavanja senzora\n");
		}

		// Odredjivanje adrese senzora na osnovu pritisnutog tastera LED bara //
		if (senzorOcitavanje == (uint8_t)1)
		{
			adresaSenzora[5] = '7';
		}
		else if (senzorOcitavanje == (uint8_t)2)
		{
			adresaSenzora[5] = '8';
		}
		else if (senzorOcitavanje == (uint8_t)4)
		{
			adresaSenzora[5] = '9';
		}
		else
		{
			adresaSenzora[5] = '0';
		}
	
		if (adresaSenzora[5] != '0')
		{
			if (xQueueSend(SensorAddress_q, &adresaSenzora, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_q");
			}
			if (xQueueSend(SensorAddress_7seg_q, &adresaSenzora, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_7seg_q");
			}
		}
	}
}

// Task koji sluzi za prijem LIN poruka od akumulatora, koje zapravo predstavljaju vrijednost senzora 
/*static void SerialReceive_Task0(void* pvParameters)
{
	uint8_t cc;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore0, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 0\n");
		}

		if (get_serial_character(COM_CH_0, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 0\n");
		}

		if (cc == (uint8_t)'S')
		{
			recPoint_0 = 0;
		}
		else if (cc == (uint8_t)'.')
		{
			if (xQueueSend(SensorAkumulatorVrijednost_q, &recBuffer_0, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red\n");
			}

			recBuffer_0[0] = '\0';
			recBuffer_0[1] = '\0';
			recBuffer_0[2] = '\0';
			recBuffer_0[3] = '\0';
			recBuffer_0[4] = '\0';
			recBuffer_0[5] = '\0';
			recBuffer_0[6] = '\0';
			recBuffer_0[7] = '\0';
			recBuffer_0[8] = '\0';
			recBuffer_0[9] = '\0';
			recBuffer_0[10] = '\0';
			recBuffer_0[11] = '\0';
			recBuffer_0[12] = '\0';
			recBuffer_0[13] = '\0';
			recBuffer_0[14] = '\0';
			recBuffer_0[15] = '\0';
		}
		else
		{
			recBuffer_0[recPoint_0++] = cc;
		}
	}
}*/

// Task koji sluzi za simulaciju akumulatora, koji salje vrijednost senzora svakih 100ms
// Realizovan tako sto se svakih 100ms salje adresa senzora
/*static void SerialSend_Task0(void* pvParameters)
{
	static uint8_t adresaSenzora = (uint8_t)'2';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
		
		if (send_serial_character(COM_CH_0, adresaSenzora) != 0)
		{
			printf("Greska prilikom slanja adrese senzora\n");
		}
	}
}*/

// Task koji se koristi za prijem adrese senzora sa PC-a.
// Ovaj task ujedno ima i provjeru za LIN poruku tako da automatski provjerava da li pristigla poruka ima oblik LIN poruke (npr. 005507)
static void SerialReceive_Task1(void* pvParameters)
{
	uint8_t cc;
	uint8_t sinhPauza = (uint8_t)0;
	uint8_t sinhronizacija = (uint8_t)0;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore1, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 1\n");
		}

		if (get_serial_character(COM_CH_1, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 1\n");
		}

		if ( (cc == (uint8_t)'0') && (sinhPauza < (uint8_t)2) )
		{
			recPoint_1 = 0;
			recBuffer_1[sinhPauza] = '0';
			sinhPauza++;
		}
		else if ( (cc == (uint8_t)'5') && (sinhPauza == (uint8_t)2) && (sinhronizacija < (uint8_t)2) )
		{
			recPoint_1 = 4;
			recBuffer_1[sinhPauza + sinhronizacija] = '5';
			sinhronizacija++;
		}
		else if ( (cc == (uint8_t)'3') && (sinhPauza == (uint8_t)2) && (sinhronizacija == (uint8_t)2) && (recPoint_1 == (uint8_t)7) )
		{
			recBuffer_1[6] = '\0';

			if (xQueueReset(SensorVrijednost_q) != pdTRUE)
			{
				printf("Neuspjesno brisanje reda\n");
			}

			if (xQueueSend(SensorAddress_q, &recBuffer_1, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red\n");
			}

			if (xQueueSend(SensorAddress_7seg_q, &recBuffer_1, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_LED_q");
			}

			if (xSemaphoreGive(ResponseToPC_BinSemaphore) != pdTRUE)
			{
				printf("Greska pri slanju semafora\n");
			}

			sinhronizacija = (uint8_t)0;
			sinhPauza = (uint8_t)0;

			recBuffer_1[0] = '\0';
			recBuffer_1[1] = '\0';
			recBuffer_1[2] = '\0';
			recBuffer_1[3] = '\0';
			recBuffer_1[4] = '\0';
			recBuffer_1[5] = '\0';
			recBuffer_1[6] = '\0';
			recBuffer_1[7] = '\0';
		}
		else
		{
			recBuffer_1[recPoint_1] = (char)cc;
			recPoint_1++;
		}
	}
}

// Task koji se koristi za slanje vrijednosti senzora na PC.
// Kad se sa PC-a posalje adresa senzora, odnosno zatrazi vrijednost odredjenog senzora, ovim taskom se ta vrijednost salje na PC
static void SerialSend_Task1(void* pvParameters)
{
	t_point = (uint8_t)0;

	for (;;)
	{
		if (t_point > (uint8_t)(sizeof(podaciZaPC) - 1))
		{
			t_point = (uint8_t)0;

			if (xSemaphoreTake(SendToPC_BinSemaphore, portMAX_DELAY) != pdTRUE)
			{
				printf("Greska prilikom preuzimanja semafora\n");
			}
		}
		else
		{
			if (send_serial_character(COM_CH_1, podaciZaPC[t_point]) != 0)
			{
				printf("Greska prilikom slanja na serijsku kanal 1\n");
			}
			t_point++;

			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

// Task koji se koristi za prijem vrijednosti senzora. 
// Ovaj senzor ima adresu 0x07 i simuliran je preko serijske komunikacije odnosno AdvUniCom kanala 2
static void SerialReceive_Task2(void* pvParameters)
{
	uint8_t cc;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore2, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 2\n");
		}

		if (get_serial_character(COM_CH_2, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 2\n");
		}

		if (cc == (uint8_t)'S')
		{
			recPoint_2 = (uint8_t)0;
		}
		else if (cc == (uint8_t)'.')
		{
			if (xQueueSend(SensorVrijednost_q, &recBuffer_2, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red\n");
			}

			if (xQueueSend(SensorVrijednost_7seg_q, &recBuffer_2, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_LED_q");
			}

			recBuffer_2[0] = '\0';
			recBuffer_2[1] = '\0';
			recBuffer_2[2] = '\0';
			recBuffer_2[3] = '\0';
			recBuffer_2[4] = '\0';
			recBuffer_2[5] = '\0';
			recBuffer_2[6] = '\0';
			recBuffer_2[7] = '\0';
			recBuffer_2[8] = '\0';
			recBuffer_2[9] = '\0';
			recBuffer_2[10] = '\0';
			recBuffer_2[11] = '\0';
			recBuffer_2[12] = '\0';
			recBuffer_2[13] = '\0';
			recBuffer_2[14] = '\0';
			recBuffer_2[15] = '\0';
		}
		else
		{
			recBuffer_2[recPoint_2] = (char)cc;
			recPoint_2++;
		}
	}
}

// Task koji se koristi za simulaciju slanja senzora.
// Kada se salje adresa 0x07, ovaj task salje broj 7 na serijsku komunikaciju kanal 2, tako da se moze simulirati
// automatski odgovor koji predstavlja vrijednost senzora
static void SerialSend_Task2(void* pvParameters)
{
	char rec_buf[7];
	const uint8_t c = (uint8_t)'7';

	for (;;)
	{
		if (xSemaphoreTake(SensID_7_BinSemaphore, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 2\n");
		}

		if (send_serial_character(COM_CH_2, c) != 0)
		{
			printf("Greska prilikom slanja adrese senzora\n");
		}
	}
}

// Task koji se koristi za prijem vrijednosti senzora. 
// Ovaj senzor ima adresu 0x08 i simuliran je preko serijske komunikacije odnosno AdvUniCom kanala 3
static void SerialReceive_Task3(void* pvParameters)
{
	uint8_t cc;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore3, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 3\n");
		}

		if (get_serial_character(COM_CH_3, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 3\n");
		}

		if (cc == (uint8_t)'S')
		{
			recPoint_3 = (uint8_t)0;
		}
		else if (cc == (uint8_t)'.')
		{
			if (xQueueSend(SensorVrijednost_q, &recBuffer_3, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red 8\n");
			}

			if (xQueueSend(SensorVrijednost_7seg_q, &recBuffer_3, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_LED_q");
			}

			recBuffer_3[0] = '\0';
			recBuffer_3[1] = '\0';
			recBuffer_3[2] = '\0';
			recBuffer_3[3] = '\0';
			recBuffer_3[4] = '\0';
			recBuffer_3[5] = '\0';
			recBuffer_3[6] = '\0';
			recBuffer_3[7] = '\0';
			recBuffer_3[8] = '\0';
			recBuffer_3[9] = '\0';
			recBuffer_3[10] = '\0';
			recBuffer_3[11] = '\0';
			recBuffer_3[12] = '\0';
			recBuffer_3[13] = '\0';
			recBuffer_3[14] = '\0';
			recBuffer_3[15] = '\0';
		}
		else
		{
			recBuffer_3[recPoint_3] = (char)cc;
			recPoint_3++;
		}
	}
}

// Task koji se koristi za simulaciju slanja senzora.
// Kada se salje adresa 0x08, ovaj task salje broj 7 na serijsku komunikaciju kanal 3, tako da se moze simulirati
// automatski odgovor koji predstavlja vrijednost senzora
static void SerialSend_Task3(void* pvParameters)
{
	char rec_buf[7];
	const uint8_t c = (uint8_t)'8';

	for (;;)
	{
		if (xSemaphoreTake(SensID_8_BinSemaphore, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 4\n");
		}

		if (send_serial_character(COM_CH_3, c) != 0)
		{
			printf("Greska prilikom slanja adrese senzora\n");
		}
	}
}

// Task koji se koristi za prijem vrijednosti senzora. 
// Ovaj senzor ima adresu 0x09 i simuliran je preko serijske komunikacije odnosno AdvUniCom kanala 4
static void SerialReceive_Task4(void* pvParameters)
{
	uint8_t cc;

	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore4, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 4\n");
		}

		if (get_serial_character(COM_CH_4, &cc) != 0)
		{
			printf("Greska pri prijemu karaktera na kanalu 4\n");
		}

		if (cc == (uint8_t)'S')
		{
			recPoint_4 = (uint8_t)0;
		}
		else if (cc == (uint8_t)'.')
		{
			if (xQueueSend(SensorVrijednost_q, &recBuffer_4, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red\n");
			}

			if (xQueueSend(SensorVrijednost_7seg_q, &recBuffer_4, 0) != pdTRUE)
			{
				printf("Greska prilikom upisa u red SensorAddress_LED_q");
			}

			recBuffer_4[0] = '\0';
			recBuffer_4[1] = '\0';
			recBuffer_4[2] = '\0';
			recBuffer_4[3] = '\0';
			recBuffer_4[4] = '\0';
			recBuffer_4[5] = '\0';
			recBuffer_4[6] = '\0';
			recBuffer_4[7] = '\0';
			recBuffer_4[8] = '\0';
			recBuffer_4[9] = '\0';
			recBuffer_4[10] = '\0';
			recBuffer_4[11] = '\0';
			recBuffer_4[12] = '\0';
			recBuffer_4[13] = '\0';
			recBuffer_4[14] = '\0';
			recBuffer_4[15] = '\0';
		}
		else
		{
			recBuffer_4[recPoint_4] = (char)cc;
			recPoint_4++;
		}
	}
}

// Task koji se koristi za simulaciju slanja senzora.
// Kada se salje adresa 0x09, ovaj task salje broj 7 na serijsku komunikaciju kanal 4, tako da se moze simulirati
// automatski odgovor koji predstavlja vrijednost senzora
static void SerialSend_Task4(void* pvParameters)
{
	char rec_buf[7];
	const uint8_t c = (uint8_t)'9';

	for (;;)
	{
		if (xSemaphoreTake(SensID_9_BinSemaphore, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora na kanalu 4\n");
		}

		if (send_serial_character(COM_CH_4, c) != 0)
		{
			printf("Greska prilikom slanja adrese senzora\n");
		}
	}
}

// Task za parsiranje. Ovaj task se koristi za prijem adrese senzora (sa PC-a ili tastera LED bara), koji na osnovu
// primljene adrese dodjeljuje odredjen binarni semafor
static void parsiranje_tsk(void* pvParameters)
{
	char rec_buf[7];

	for (;;)
	{
		if (xQueueReceive(SensorAddress_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if ( (rec_buf[4] == '0') && (rec_buf[5] == '7') )
		{
			if (xSemaphoreGive(SensID_7_BinSemaphore) != pdTRUE)
			{
				printf("Greska pri dodjeli vrijednosti semafora\n");
			}
		}
		else if ( (rec_buf[4] == '0') && (rec_buf[5] == '8') )
		{
			if (xSemaphoreGive(SensID_8_BinSemaphore) != pdTRUE)
			{
				printf("Greska pri dodjeli vrijednosti semafora\n");
			}
		}
		else if ( (rec_buf[4] == '0') && (rec_buf[5] == '9') )
		{
			if (xSemaphoreGive(SensID_9_BinSemaphore) != pdTRUE)
			{
				printf("Greska pri dodjeli vrijednosti semafora\n");
			}
		}
		else
		{
			printf("POGRESNO UNESENA VRIJEDNOST ADRESE");
		}
	}
}

// Task koji ispisuje poslednju zatrazenu adresu senzora na 7seg displej
static void ispisAdreseNa7seg_tsk(void* pvParameters)
{
	char rec_buf[20];

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));

		if (xQueueReceive(SensorAddress_7seg_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if (select_7seg_digit(0) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[4]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(1) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[5]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
	}
}

// Task koji ispisuje vrijednost senzora sa zatrazene adrese na 7seg displej
static void ispisVrijednostiSenzoraNa7seg_tsk(void* pvParameters)
{
	char rec_buf[17];

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));

		if (xQueueReceive(SensorVrijednost_7seg_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if (select_7seg_digit(2) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[0]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(3) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[1]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(4) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[2]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(5) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[3]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(6) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[4]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(7) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[5]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(8) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[6]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
		if (select_7seg_digit(9) != 0)
		{
			printf("Greska prilikom izbora cifre 7seg displeja\n");
		}
		if (set_7seg_digit(hexnum[(uint8_t)rec_buf[7]]) != 0)
		{
			printf("Greska prilikom ispisa na 7seg displej\n");
		}
	}
}

// Task za slanje vrijednosti senzora na PC. Kad primi red, smjesta ga u globalnu promjenljivu 
// koja se onda preko taska SerialSent_Task1 salje na PC.
// On takodje ceka da dobije binarni semafor od taska za prijem poruka od PC-a. Ovo je potrebno
// da ne bi doslo do zabune odnosno da se u globalnu promjenljivu ne smjesti podatak dobijen
// kada je vrijednost zatrazena od senzora sa tastera LED bara
static void slanjeNaPC_tsk(void* pvParameters)
{
	for (;;)
	{
		if (xSemaphoreTake(ResponseToPC_BinSemaphore, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska prilikom preuzimanja semafora\n");
		}

		podaciZaPC[0] = '\n';
		podaciZaPC[1] = '\n';
		podaciZaPC[2] = '\n';
		podaciZaPC[3] = '\n';
		podaciZaPC[4] = '\n';
		podaciZaPC[5] = '\n';
		podaciZaPC[6] = '\n';
		podaciZaPC[7] = '\n';
		podaciZaPC[8] = '\n';
		podaciZaPC[9] = '\n';
		podaciZaPC[10] = '\n';
		podaciZaPC[11] = '\n';
		podaciZaPC[12] = '\n';
		podaciZaPC[13] = '\n';
		podaciZaPC[14] = '\n';
		podaciZaPC[15] = '\n';

		if (xQueueReceive(SensorVrijednost_q, &podaciZaPC, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		xSemaphoreGive(SendToPC_BinSemaphore);
	}
}

// Interrupt rutina za serijsku komunikaciju u kojoj se provjerava koji je kanal poslao i na osnovu toga daje odgovarajuci semafor //
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	/*if (get_RXC_status(0) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore0, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}*/
	if (get_RXC_status(1) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	if (get_RXC_status(2) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore2, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	if (get_RXC_status(3) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore3, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	if (get_RXC_status(4) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore4, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}

// MAIN - SYSTEM STARTUP POINT //
void main_demo(void)
{
	// Inicijalizacija periferija //
	if (init_LED_comm() != 0)
	{
		printf("Neuspjesna inicijalizacija\n");
	}
	if (init_7seg_comm() != 0)
	{
		printf("Neupsjesna inicijalizacija\n");
	}

	// Inicijalizacija serijske TX na kanalu 0 //
	/*if (init_serial_uplink(COM_CH_0) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 0\n");
	}
	// Inicijalizacija serijske RX na kanalu 0 //
	if (init_serial_downlink(COM_CH_0) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 0\n");
	}*/
	// Inicijalizacija serijske TX na kanalu 1 //
	if (init_serial_uplink(COM_CH_1) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 1\n");
	}
	// Inicijalizacija serijske RX na kanalu 1 //
	if (init_serial_downlink(COM_CH_1) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 1\n");
	}
	// Inicijalizacija serijske TX na kanalu 2 //
	if (init_serial_uplink(COM_CH_2) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 2\n");
	}
	// Inicijalizacija serijske RX na kanalu 2 //
	if (init_serial_downlink(COM_CH_2) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 2\n");
	}
	// Inicijalizacija serijske TX na kanalu 3 //
	if (init_serial_uplink(COM_CH_3) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 3\n");
	}
	// Inicijalizacija serijske RX na kanalu 3 //
	if (init_serial_downlink(COM_CH_3) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 3\n");
	}
	// Inicijalizacija serijske TX na kanalu 4 //
	if (init_serial_uplink(COM_CH_4) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 4\n");
	}
	// Inicijalizacija serijske RX na kanalu 4 //
	if (init_serial_downlink(COM_CH_4) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 4\n");
	}

	// SERIAL RECEPTION INTERRUPT HANDLER //
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	// Kreiranje semafora //
	RXC_BinarySemaphore0 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore0 == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	RXC_BinarySemaphore1 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore1 == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	RXC_BinarySemaphore2 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore2 == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	RXC_BinarySemaphore3 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore3 == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	RXC_BinarySemaphore4 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore4 == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	SensID_7_BinSemaphore = xSemaphoreCreateBinary();
	if (SensID_7_BinSemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	SensID_8_BinSemaphore = xSemaphoreCreateBinary();
	if (SensID_8_BinSemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	SensID_9_BinSemaphore = xSemaphoreCreateBinary();
	if (SensID_9_BinSemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	SendToPC_BinSemaphore = xSemaphoreCreateBinary();
	if (SendToPC_BinSemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}
	ResponseToPC_BinSemaphore = xSemaphoreCreateBinary();
	if (ResponseToPC_BinSemaphore == NULL)
	{
		printf("Greska prilikom kreiranja semafora\n");
	}

	// Kreiranje redova //
	/*SensorAkumulatorVrijednost_q = xQueueCreate(4, 16U);
	if (SensorAkumulatorVrijednost_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}*/
	SensorAddress_q = xQueueCreate(4, 16U);
	if (SensorAddress_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorAddress_7seg_q = xQueueCreate(4, 16U);
	if (SensorAddress_7seg_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorVrijednost_q = xQueueCreate(4, 16U);
	if (SensorVrijednost_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorVrijednost_PC_q = xQueueCreate(4, 16U);
	if (SensorVrijednost_PC_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorVrijednost_7seg_q = xQueueCreate(4, 16U);
	if (SensorVrijednost_7seg_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
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
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

	status = xTaskCreate(
		slanjeNaPC_tsk,
		"lanjeNaPC_tsk",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

	status = xTaskCreate(
		parsiranje_tsk,
		"parsiranje_tsk",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

	status = xTaskCreate(
		ispisAdreseNa7seg_tsk,
		"ispisAdreseNa7seg_tsk",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

	status = xTaskCreate(
		ispisVrijednostiSenzoraNa7seg_tsk,
		"ispisVrijednostiSenzoraNa7seg_tsk",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

	// SERIAL RECEIVER AND SEND TASK //
	/*status = xTaskCreate(SerialReceive_Task0, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task0, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}*/
	status = xTaskCreate(SerialReceive_Task1, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task1, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialReceive_Task2, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task2, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialReceive_Task3, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task3, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialReceive_Task4, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task4, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

    vTaskStartScheduler();

}