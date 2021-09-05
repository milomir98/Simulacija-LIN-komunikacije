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
#define COM_CH_2 (2)
#define COM_CH_3 (3)
#define COM_CH_4 (4)

/* TASK PRIORITIES */
#define SERVICE_TASK_PRI		(tskIDLE_PRIORITY + (UBaseType_t)1)
#define TASK_SERIAL_SEND_PRI	(tskIDLE_PRIORITY + (UBaseType_t)2)
#define TASK_SERIAL_REC_PRI		(tskIDLE_PRIORITY + (UBaseType_t)3)

/* TASKS: FORWARD DECLARATIONS */
static void led_bar_tsk(void* pvParameters);
static void parsiranje_tsk(void* pvParameters);
static void SerialSend_Task0(void* pvParameters);
static void SerialReceive_Task0(void* pvParameters);
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
static uint8_t volatile adresaSenzora;

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
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t RXC_BinarySemaphore0;
static SemaphoreHandle_t RXC_BinarySemaphore1;
static SemaphoreHandle_t RXC_BinarySemaphore2;
static SemaphoreHandle_t RXC_BinarySemaphore3;
static SemaphoreHandle_t RXC_BinarySemaphore4;
static QueueHandle_t SensorValue_q;
static QueueHandle_t SensorAddress_LED_q;
static QueueHandle_t SensorVrijednost_q;

// Task koji na osnovu pritisnutog tastera na led baru ispisuje na 7seg displej informaciju, brzina osvjezavanja je 100ms
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
		if (senzorOcitavanje == 1)
		{
			adresaSenzora[5] = '7';
		}
		else if (senzorOcitavanje == 2)
		{
			adresaSenzora[5] = '8';
		}
		else if (senzorOcitavanje == 4)
		{
			adresaSenzora[5] = '9';
		}
		else
		{
			adresaSenzora[5] = '0';
		}
	
		if (xQueueSend(SensorAddress_LED_q, &adresaSenzora, 0) != pdTRUE)
		{
			printf("Greska prilikom upisa u red SensorAddress_LED_q");
		}
		//printf("%s", adresaSenzora);
	}
}

// Task koji sluzi za prijem LIN poruka od akumulatora, koje zapravo predstavljaju vrijednost senzora 
static void SerialReceive_Task0(void* pvParameters)
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
			if (xQueueSend(SensorValue_q, &recBuffer_0, 0) != pdTRUE)
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
}

// Task koji sluzi za simulaciju akumulatora, koji salje vrijednost senzora svakih 100ms
// Realizovan tako sto se svakih 100ms salje adresa senzora
static void SerialSend_Task0(void* pvParameters)
{
	//strcpy(&adresaSenzora, "CA");	//Ovo treba izmjeniti
	adresaSenzora = (uint8_t)'2';

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
		
		if (send_serial_character(COM_CH_0, adresaSenzora) != 0)
		{
			printf("Greska prilikom slanja adrese senzora\n");
		}
	}
}

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
			recPoint_2 = 0;
		}
		else if (cc == (uint8_t)'.')
		{
			if (xQueueSend(SensorVrijednost_q, &recBuffer_2, 0) != pdTRUE)
			{
				printf("Neuspjesno slanje podataka u red\n");
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
			recBuffer_2[recPoint_2++] = cc;
		}
	}
}

static void SerialSend_Task2(void* pvParameters)
{
	char rec_buf[7];
	uint8_t c = (uint8_t)'7';

	for (;;)
	{
		//vTaskDelay(pdMS_TO_TICKS(100));
		if (xQueueReceive(SensorAddress_LED_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if (rec_buf[5] == '7')
		{
			if (send_serial_character(COM_CH_2, c) != 0)
			{
				printf("Greska prilikom slanja adrese senzora\n");
			}
		}
	}
}

static void SerialReceive_Task3(void* pvParameters)
{

}

static void SerialSend_Task3(void* pvParameters)
{
	char rec_buf[7];
	uint8_t c = (uint8_t)'8';

	for (;;)
	{
		//vTaskDelay(pdMS_TO_TICKS(100));
		if (xQueueReceive(SensorAddress_LED_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if (rec_buf[5] == '8')
		{
			if (send_serial_character(COM_CH_3, c) != 0)
			{
				printf("Greska prilikom slanja adrese senzora\n");
			}
		}
	}
}

static void SerialReceive_Task4(void* pvParameters)
{

}

static void SerialSend_Task4(void* pvParameters)
{
	char rec_buf[7];
	uint8_t c = (uint8_t)'9';

	for (;;)
	{
		//vTaskDelay(pdMS_TO_TICKS(100));
		if (xQueueReceive(SensorAddress_LED_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}

		if (rec_buf[5] == '9')
		{
			if (send_serial_character(COM_CH_4, c) != 0)
			{
				printf("Greska prilikom slanja adrese senzora\n");
			}
		}
	}
}

// Task za parsiranje, nakon prijema adrese, prosledjuje je dalje akumulatoru koji onda vraca vrijednost senzora
static void parsiranje_tsk(void* pvParameters)
{
	char rec_buf[6] = { 0 };

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
		if (xQueueReceive(SensorVrijednost_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Greska pri preuzimanju vrijednosti iz reda\n");
		}
		printf("Vrijednost rec_buf je %s", rec_buf);
	}
}

// Interrupt rutina za serijsku komunikaciju u kojoj se provjerava koji je kanal poslao i na osnovu toga daje odgovarajuci semafor //
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (get_RXC_status(0) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore0, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	/*if (get_RXC_status(1) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}*/
	if (get_RXC_status(2) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore2, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	/*if (get_RXC_status(3) != 0)
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
	}*/
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
	if (init_serial_uplink(COM_CH_0) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 0\n");
	}
	// Inicijalizacija serijske RX na kanalu 0 //
	if (init_serial_downlink(COM_CH_0) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 0\n");
	}
	// Inicijalizacija serijske TX na kanalu 1 //
	/*if (init_serial_uplink(COM_CH_1) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 1\n");
	}
	// Inicijalizacija serijske RX na kanalu 1 //
	if (init_serial_downlink(COM_CH_1) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 1\n");
	}*/
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
	/*// Inicijalizacija serijske RX na kanalu 3 //
	if (init_serial_downlink(COM_CH_3) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 3\n");
	}*/
	// Inicijalizacija serijske TX na kanalu 4 //
	if (init_serial_uplink(COM_CH_4) != 0)
	{
		printf("Neuspjesna inicijalizacija TX na kanalu 4\n");
	}
	// Inicijalizacija serijske RX na kanalu 4 //
	/*if (init_serial_downlink(COM_CH_4) != 0)
	{
		printf("Neuspjesna inicijalizacija RX na kanalu 4\n");
	}*/

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

	// Kreiranje reda //
	SensorValue_q = xQueueCreate(4, 16U);
	if (SensorValue_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorAddress_LED_q = xQueueCreate(4, 16U);
	if (SensorAddress_LED_q == NULL)
	{
		printf("Greska prilikom kreiranja reda\n");
	}
	SensorVrijednost_q = xQueueCreate(4, 16U);
	if (SensorVrijednost_q == NULL)
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

	// SERIAL RECEIVER AND SEND TASK //
	status = xTaskCreate(SerialReceive_Task0, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task0, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	/*status = xTaskCreate(SerialReceive_Task1, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	status = xTaskCreate(SerialSend_Task1, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}*/
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
	/*status = xTaskCreate(SerialReceive_Task3, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}*/
	status = xTaskCreate(SerialSend_Task3, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}
	/*status = xTaskCreate(SerialReceive_Task4, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}*/
	status = xTaskCreate(SerialSend_Task4, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja taska\n");
	}

    vTaskStartScheduler();

}