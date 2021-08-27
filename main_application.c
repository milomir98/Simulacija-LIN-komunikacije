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

/* DEFINISAN PUN REZERVOAR U LITRIMA */
#define PUN_REZERVOAR 40

/* TASK PRIORITIES */
#define	TASK_SERIAL_SEND_PRI		(tskIDLE_PRIORITY + (UBaseType_t)2 ) 
#define TASK_SERIAL_REC_PRI			( tskIDLE_PRIORITY + (UBaseType_t)3 )
#define	SERVICE_TASK_PRI		( tskIDLE_PRIORITY + (UBaseType_t)1 ) 

/* TASKS: FORWARD DECLARATIONS */

static void led_bar_tsk(void* pvParameters);
static void merenje_proseka_nivoa_goriva(void* pvParameters);
static void nivo_goriva_u_procentima(void* pvParameters);
static void SerialSend_Task0(void* pvParameters);
static void SerialSend_Task1(void* pvParameters);
static void SerialReceive_Task0(void* pvParameters);
static void SerialReceive_Task1(void* pvParameters);
void main_demo(void);

/* TRASNMISSION DATA - CONSTANT IN THIS APPLICATION */
static uint16_t MINFUEL;
static uint16_t POTROSNJA;
static uint16_t MAXFUEL;
static uint16_t nivo_goriva_procenti;
static uint16_t autonomija;
static uint16_t otpornost;

/* RECEPTION DATA BUFFER */
#define R_BUF_SIZE (32)
static char r_buffer[R_BUF_SIZE]; //bio uint8
static char r_buffer1[R_BUF_SIZE];
static char brojevi[R_BUF_SIZE];
static char min_otpornost[R_BUF_SIZE];
static char max_otpornost[R_BUF_SIZE];
static uint8_t volatile r_point,r_point1;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* GLOBAL OS-HANDLES */
static SemaphoreHandle_t RXC_BinarySemaphore0;
static SemaphoreHandle_t RXC_BinarySemaphore1;
static QueueHandle_t otpornost_q;

/* Task koji na osnovu pritisnutog tastera ispisuje na 7seg displej informacije, brzina osvezavanja 100ms
   Pritiskom na jedan taster ispisuje nivo goriva u procentima i otpornost
   Pritiskom na drugi taster ispisuje koliko jos kilometara moze da se krece i koliki je rezultat START-STOP naredbe
   START i STOP su realizovani preko tastera  */

static void led_bar_tsk( void* pvParameters)
{
	uint8_t d,s;
	uint16_t start_procenti=0;
	uint16_t stop_procenti=0;
	uint16_t razlika=0;

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));

		if (get_LED_BAR(2, &d) != 0)
		{
			printf("Greska prilikom ocitavanja");
		}
		if (get_LED_BAR(3, &s) != 0)
		{
			printf("Greska prilikom ocitavanja");
		}

		if (d == (uint8_t)1)
		{
			uint16_t jedinica = nivo_goriva_procenti % (uint16_t) 10;
			uint16_t desetica = (nivo_goriva_procenti / (uint16_t)10) % (uint16_t)10;
			uint16_t stotina = nivo_goriva_procenti / (uint16_t)100;
			if (select_7seg_digit(2) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[jedinica]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(1) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[desetica]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(0) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[stotina]) != 0)
			{
				printf("Problem");
			}

			uint16_t jedinica_o = otpornost % (uint16_t)10;
			uint16_t desetica_o = (otpornost / (uint16_t)10) % (uint16_t)10;
			uint16_t stotina_o = (otpornost / (uint16_t)100) % (uint16_t)10;
			uint16_t hiljada_o = (otpornost / (uint16_t)1000) % (uint16_t)10;
			uint16_t broj_o = otpornost / (uint16_t)10000;
			if (select_7seg_digit(8) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[jedinica_o]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(7) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[desetica_o]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(6) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[stotina_o]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(5) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[hiljada_o]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(4) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[broj_o]) != 0)
			{
				printf("Problem");
			}
		}
		else if (d == (uint8_t)2)
		{
			
			uint16_t jedinica = autonomija % (uint16_t)10; 
			uint16_t desetica = (autonomija / (uint16_t)10) % (uint16_t)10;
			uint16_t stotina = autonomija / (uint16_t)100;
			if (select_7seg_digit(2) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[jedinica]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(1) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[desetica]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(0) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[stotina]) != 0)
			{
				printf("Problem");
			}

			uint16_t jedinica_r = razlika % (uint16_t)10;
			uint16_t desetica_r = razlika / (uint16_t)10;
			if (select_7seg_digit(8) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[jedinica_r]) != 0)
			{
				printf("Problem");
			}
			if (select_7seg_digit(7) != 0)
			{
				printf("Problem");
			}
			if (set_7seg_digit(hexnum[desetica_r]) != 0)
			{
				printf("Problem");
			}
		}
		else
		{
			for (int i = 0; i < 9; i++)
			{
				if (select_7seg_digit((uint8_t)i) != 0)
				{
					printf("Problem");
				}
				if (set_7seg_digit(0x00) != 0)
				{
					printf("Problem");
				}
			}
		}
		//aktivan start
		if (s == (uint8_t)128)
		{
			start_procenti = nivo_goriva_procenti;
			if (set_LED_BAR(1, 0x08) != 0)
			{
				printf("Problem");
			}
		}
		//aktivan stop
		else if (s == (uint8_t)64)
		{
			if (set_LED_BAR(1, 0x00) != 0)
			{
				printf("Problem");
			}
			stop_procenti = nivo_goriva_procenti;
			razlika = start_procenti - stop_procenti;
		}
		else
		{
			printf("Nije pritisnut ni stop ni start");
		}
		
	}
}

/* Task koji ima za zadatak da meri prosek zadnjih pet pristiglih vrednosti otpornosti */
static void merenje_proseka_nivoa_goriva(void* pvParameters)
{
	int rec_buf,brojac=0,suma=0,prosek;

	for (;;) 
	{
		if (xQueueReceive(otpornost_q, &rec_buf, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesan prijem");
		}
		suma = suma + rec_buf;
		brojac++;
		if (brojac == 5)
		{
			prosek = suma / 5;
			//printf("Prosek %d\n", prosek);
			brojac = 0;
			suma = 0;
		}

	}
}

/* Task koji ima za zadatak da preracuna trenutni nivo goriva u procentima i izracuna koliko jos km moze automobil da se krece
sa trenutnom kolicinom goriva */
static void nivo_goriva_u_procentima(void* pvParameters)
{
	uint16_t rec_buf;
	uint8_t str[3];

	for (;;)
	{
		if(xQueueReceive(otpornost_q, &rec_buf, portMAX_DELAY)!=pdTRUE)
		{
			printf("Neuspesan prijem");
		}
		
		/* Tek kada se unesu svi parametri, MINFUEL, MAXFUEL, PP, ispisuje se i racuna se */
		if (MINFUEL != (uint16_t) 0 && MAXFUEL != (uint16_t) 0)
		{
			nivo_goriva_procenti = (uint16_t)100 * (rec_buf - MINFUEL) / (MAXFUEL - MINFUEL);
			printf("U procentima %d %% \n", nivo_goriva_procenti);
			if (nivo_goriva_procenti < (uint16_t) 10)
			{
				if(set_LED_BAR(0, 0x01)!=0)
				{
					printf("Problem");
				}
			}
			else
			{
				if(set_LED_BAR(0, 0x00)!=0)
				{
					printf("Problem");
				}
			}
		}
		if (POTROSNJA != (uint16_t) 0 && MINFUEL != (uint16_t) 0 && MAXFUEL != (uint16_t) 0)
		{
			autonomija = nivo_goriva_procenti * (uint16_t)PUN_REZERVOAR / POTROSNJA;
			printf("Moze jos %d km\n", autonomija);
		}
	}
}

/* Task koji vrsi prijem podataka sa kanala 0 - stizu vrednosti otpornosti sa senzora
   Format poruke: R7000.  (u ovom primeru je 7000 vrednost otpornosti)  */
static void SerialReceive_Task0(void* pvParameters)
{
	uint8_t cc;

	for (;;)
	{
		if(xSemaphoreTake(RXC_BinarySemaphore0, portMAX_DELAY)!=pdTRUE)
		{
			printf("Greska");
		}
		
		if(get_serial_character(COM_CH_0, &cc)!=0)
		{
			printf("Greska");
		}

		//kada stignu podaci, salju se u red
		if (cc == (uint8_t) 'R')
		{
			r_point = 0;
		}
		else if (cc == (uint8_t)'.')
		{	
			char *ostatak;
			printf(" Otpornost je %s\n", r_buffer);
			otpornost = (uint16_t)strtol(r_buffer,&ostatak,10);
			if(xQueueSend(otpornost_q, &otpornost, 0)!= pdTRUE)
			{
				printf("Neuspesno slanje u red");
			}
			
			r_buffer[0] = '\0';
			r_buffer[1] = '\0';
			r_buffer[2] = '\0';
			r_buffer[3] = '\0';
			r_buffer[4] = '\0';
		}
		else 
		{
			r_buffer[r_point++] = (char) cc; 
		}
	
	}
}

/* Task koji vrsi prijem podataka sa kanala 1
   Naredbe koje stizu su: MINFUEL<vrednost>, MAXFUEL<vrednost>, PP<vrednost>
   Format poruke: \00<naredba>\0d 
   NPR.
       \00MINFUEL10\0d
	   \00MAXFUEL9000\0d
	   \00PP8\0d                         */
static void SerialReceive_Task1(void* pvParameters)
{
	uint8_t cc;
	uint8_t j= 0,k=0,l=0;
	
	for (;;)
	{
		if (xSemaphoreTake(RXC_BinarySemaphore1, portMAX_DELAY) != pdTRUE)
		{
			printf("Neuspesno");
		}

		if(get_serial_character(COM_CH_1, &cc)!=0)
		{
			printf("Neuspesno");
		}
		
		if (cc == (uint8_t) 0x00)
		{
			j = 0;
			k = 0;
			l = 0;
			r_point1 = 0;
		}
		else if (cc == (uint8_t)13) // 13 decimalno je CR(carriage return)
		{	
			if (r_buffer1[0] == 'M' && r_buffer1[1] == 'I' && r_buffer1[2] == 'N')
			{
				size_t i;
				for ( i = (size_t)0; i < strlen(r_buffer1); i++)
				{
					//Pristigla naredba je MINFUEL iz koje je potrebno izvuci brojeve
					//Npr. MINFUEL10
					if (r_buffer1[i] == '0' || r_buffer1[i] == '1' || r_buffer1[i] == '2' || r_buffer1[i] == '3' || r_buffer1[i] == '4' || r_buffer1[i] == '5' || r_buffer1[i] == '6' || r_buffer1[i] == '7' || r_buffer1[i] == '8' || r_buffer1[i] =='9')
					{
						min_otpornost[k] = r_buffer1[i];
						k++;
					}
				}
			}
			else if (r_buffer1[0] == 'M' && r_buffer1[1] == 'A' && r_buffer1[2] =='X')
			{
				size_t i;
				for (i =0; i < strlen(r_buffer1); i++)
				{
					//Pristigla naredba je MAXFUEL iz koje je potrebno izvuci brojeve
					//Npr. MAXFUEL9000
					if (r_buffer1[i] =='0' || r_buffer1[i] == '1' || r_buffer1[i] == '2' || r_buffer1[i] == '3' || r_buffer1[i] == '4' || r_buffer1[i] == '5' || r_buffer1[i] == '6' || r_buffer1[i] == '7' || r_buffer1[i] =='8' || r_buffer1[i] =='9')
					{
						max_otpornost[l] = r_buffer1[i];
						l++;
					}
				}
			}
			else if (r_buffer1[0] =='P' && r_buffer1[1] =='P')
			{
				size_t i;
				for (i = (size_t)0; i < strlen(r_buffer1); i++)
				{
					//Pristigla naredba je PP iz koje je potrebno izvuci brojeve
					//Npr. PP8
					if (r_buffer1[i] =='0' || r_buffer1[i] =='1' || r_buffer1[i] =='2' || r_buffer1[i] =='3' || r_buffer1[i] =='4' || r_buffer1[i] == '5' || r_buffer1[i] == '6' || r_buffer1[i] == '7' || r_buffer1[i] =='8' || r_buffer1[i] == '9')
					{
						brojevi[j] = r_buffer1[i];
						j++;
					}
				}
			}
			else
			{
				printf("Nepoznata naredba");
			}

			char *ostatak;
			MINFUEL =(uint16_t)strtol(min_otpornost,&ostatak,10);
			MAXFUEL =(uint16_t)strtol(max_otpornost,&ostatak,10);
			printf("min %d\n", MINFUEL);
			printf("max %d\n", MAXFUEL);
			POTROSNJA =(uint16_t)strtol(brojevi,&ostatak,10);
			printf("potrosnja %d\n", POTROSNJA);

			r_buffer1[0] ='\0';
			r_buffer1[1] ='\0';
			r_buffer1[2] ='\0';
			r_buffer1[3] ='\0';
			r_buffer1[4] ='\0';
			r_buffer1[5] ='\0';
			r_buffer1[6] ='\0';
			r_buffer1[7] ='\0';
			r_buffer1[8] ='\0';
			r_buffer1[9] ='\0';
			r_buffer1[10] ='\0';
			r_buffer1[11] ='\0';
		}
		else
		{
			r_buffer1[r_point1++] =(char) cc;
		}
	}
}

/* Sa ovim taskom simuliramo vrednosti otpornosti koje stizu sa senzora svakih 1s, tako sto
   svakih 1s saljemo karakter 'a' i u AdvUniCom simulatoru omogucimo tu opciju (AUTO ukljucen) */
static void SerialSend_Task0(void* pvParameters)
{
	uint8_t c= (uint8_t)'a';
	
	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (send_serial_character(COM_CH_0, c) != 0)
		{
			printf("Greska prilikom slanja");
		}
	}
}

/* Ovim taskom preko kanala 1 saljemo vrednost nivoa goriva u procentima, svakih 1s */
static void SerialSend_Task1(void* pvParameters)
{
	uint8_t str[5];
	static uint8_t brojac = 0;

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		//sprintf fja ne sme da se koristi po Misra standardu
		//sprintf(str, "%.1f%%\n",nivo_goriva_procenti); 
		/*nivo_goriva_procenti bio tipa double i format stringa koji se salje je bio npr. 33.3%\n  ali sada je tipa uint16_t.
		Tako da je format stringa koji se salje npr 89% ili 100% */

		uint16_t jedinica = nivo_goriva_procenti % (uint16_t)10;
		uint16_t desetica = (nivo_goriva_procenti / (uint16_t)10) % (uint16_t)10;
		uint16_t stotina = nivo_goriva_procenti / (uint16_t)100;

		str[0] = (uint8_t)stotina + (uint8_t)'0';
		str[1] = (uint8_t)desetica + (uint8_t)'0';
		str[2] = (uint8_t)jedinica + (uint8_t)'0';
		str[3] = (uint8_t)'%';
		str[4] = (uint8_t)'\n';

		brojac++;

		if (MINFUEL != (uint16_t)0 && MAXFUEL != (uint16_t)0)
		{
			if (brojac == (uint8_t)1)
			{
				if (send_serial_character(COM_CH_1, str[0]) != 0)
				{
					printf("Greska prilikom slanja");
				}
			}
			else if (brojac == (uint8_t)2)
			{
				if (send_serial_character(COM_CH_1, str[1]) != 0)
				{
					printf("Greska prilikom slanja");
				}
			}
			else if (brojac == (uint8_t)3)
			{
				if (send_serial_character(COM_CH_1, str[2]) != 0)
				{
					printf("Greska prilikom slanja");
				}
			}
			else if (brojac == (uint8_t)4)
			{
				if (send_serial_character(COM_CH_1, str[3]) != 0)
				{
					printf("Greska prilikom slanja");
				}
			}
			else if (brojac == (uint8_t)5)
			{
				if (send_serial_character(COM_CH_1, str[4]) != 0)
				{
					printf("Greska prilikom slanja");
				}
			}
			else
			{
				brojac = (uint8_t)0;
			}
		}
	}
}

/* Interrupt rutina za serijsku komunikaciju u kojoj se proverava koji je kanal poslao i na osnovu toga daje odgovarajuci semafor */
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
	if (get_RXC_status(1) != 0)
	{
		if (xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &higher_priority_task_woken) != pdTRUE)
		{
			printf("Greska pri slanju podatka\n");
		}
	}
	portYIELD_FROM_ISR((uint32_t)higher_priority_task_woken);
}

/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	/* Inicijalizacija periferija */
	if (init_LED_comm() != 0)
	{
		printf("Neuspesna inicijalizacija");
	}
	if(init_7seg_comm()!=0)
	{
		printf("Neuspesna inicijalizacija");
	}
	// inicijalizacija serijske TX na kanalu 0
	if(init_serial_uplink(COM_CH_0)!=0)
	{
		printf("Neuspesna inicijalizacija");
	}
	// inicijalizacija serijske TX na kanalu 1
	if(init_serial_uplink(COM_CH_1)!=0)
	{
		printf("Neuspesna inicijalizacija");
	}
	// inicijalizacija serijske RX na kanalu 0
	if(init_serial_downlink(COM_CH_0)!=0)
	{
		printf("Neuspesna inicijalizacija");
	}
	// inicijalizacija serijske RX na kanalu 1
	if(init_serial_downlink(COM_CH_1)!=0)
	{
		printf("Neuspesna inicijalizacija");
	}

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);//interrupt za serijsku prijem

	/* Kreiranje semafora */
	RXC_BinarySemaphore0 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore0 == NULL) 
	{
		printf("Greska prilikom kreiranja\n");
	}

	RXC_BinarySemaphore1 = xSemaphoreCreateBinary();
	if (RXC_BinarySemaphore1 == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*Kreiranje reda*/
	otpornost_q = xQueueCreate(10, sizeof(uint16_t));
	if (otpornost_q == NULL)
	{
		printf("Greska prilikom kreiranja\n");
	}

	/*Kreiranje taskova*/
	BaseType_t status;
	status = xTaskCreate(
		merenje_proseka_nivoa_goriva,
		"merenje task",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}

	status = xTaskCreate(
		nivo_goriva_u_procentima,
		"procenti task",
		configMINIMAL_STACK_SIZE,
		NULL,
		(UBaseType_t)SERVICE_TASK_PRI,
		NULL
	);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}

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
		printf("Greska prilikom kreiranja\n");
	}

	/* SERIAL RECEIVER AND SEND TASK */
	status=xTaskCreate(SerialReceive_Task0, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}

	status=xTaskCreate(SerialReceive_Task1, "SRx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_REC_PRI, NULL);
	if (status != pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}
	
	status=xTaskCreate(SerialSend_Task0, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status !=pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}

	status=xTaskCreate(SerialSend_Task1, "STx", configMINIMAL_STACK_SIZE, NULL, (UBaseType_t)TASK_SERIAL_SEND_PRI, NULL);
	if (status !=pdPASS)
	{
		printf("Greska prilikom kreiranja\n");
	}
	

	r_point = 0;
	r_point1 = 0;

    vTaskStartScheduler();

}