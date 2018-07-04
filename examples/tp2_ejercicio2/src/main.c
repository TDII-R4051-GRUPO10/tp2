/* Copyright 2016
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.*/


/*==================[inclusions]=============================================*/

#include "../../tp2_ejercicio3/inc/main.h"

#include "../../tp2_ejercicio3/inc/FreeRTOSConfig.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

xQueueHandle xQueue = NULL;
xQueueHandle xQueue2 = NULL;
xQueueHandle xQueue3 = NULL;

//Estructura de información el evento tecla
typedef struct _TECLA
{
	unsigned int 	codigo;
	int 			tiempo;
}TECLA;

/*==================[internal functions declaration]=========================*/


static void initHardware(void);

/*==================[internal data definition]===============================*/
#define mainQUEUE_LENGTH					( 5 )

#define REFRESH_RATE_ms 700
#define TIEMPO_DE_DIAGNOSTICO_ms 2000
#define SCAN_RATE_ms 150
#define TIEMPO_DE_DEBOUNCE_ms 20

#define MASK_REDLIGHT	4
#define MASK_GREENLIGHT	2
#define MASK_BLUELIGHT	1

#define BOTON_NO_PRESIONADO 0
#define BOTON_PRESIONADO	1

unsigned int tiempo_de_diagnostico = TIEMPO_DE_DIAGNOSTICO_ms;
int tiempo_de_pulsacion = -1;

#define NO_OPRIMIDO	0
#define DEBOUNCE	1
#define VALIDAR		2
#define OPRIMIDO	3

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
	Board_Init();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
}

void JuegoDeLuces(void *pvParameters)
{
	unsigned int estado_de_salidas = 0; //random
	unsigned int sentido  = 0;
	unsigned int continua = 1;
	TECLA tecla;
	portTickType xMeDesperte;
	xMeDesperte = xTaskGetTickCount();

	//Loop infinito
	while(1)
	{
	    //chequeo si debe cambiar el sentido
		if(xQueueReceive( xQueue, &tecla, 0 ) == pdTRUE)
			sentido^=0x01;


	    //chequeo si debe parar la secuencia
		if(xQueueReceive( xQueue2, &tecla, 0 ) == pdTRUE)
			continua=0;

	    //chequeo si debe seguir la secuencia
		if(xQueueReceive( xQueue3, &tecla, 0 ) == pdTRUE)
			continua=1;

		if(continua){
		if (!sentido)
			estado_de_salidas++;
		else
			estado_de_salidas--;

		switch(estado_de_salidas)
		{
		case 8:
			estado_de_salidas = 0x1;
			break;
		case 0:
			estado_de_salidas = 0x7;
			break;
		}

		//Red activity
		if (estado_de_salidas&MASK_REDLIGHT)
			Board_LED_Set(3, 1);
		else
			Board_LED_Set(3, 0);

		//Green activity
		if (estado_de_salidas&MASK_GREENLIGHT)
			Board_LED_Set(4, 1);
		else
			Board_LED_Set(4, 0);

		//Blue activity
		if (estado_de_salidas&MASK_BLUELIGHT)
			Board_LED_Set(5, 1);
		else
			Board_LED_Set(5, 0);


		//Con el delay de la tarea, la suspendo hasta finalizar el tiempo
		vTaskDelayUntil(&xMeDesperte,REFRESH_RATE_ms/portTICK_RATE_MS);

	}}

}

void Diagnostico(void *pvParameters)
{

	// vPrintString("Comienza el diagnostico\n");
	//Loop infinito
	while(1)
	{
		//IMPORTANTE:
		//Este loop pretende simular un conjunto de funciones que
		//insumen un tiempo para realizar el diagnostico del equipo
		//Pero NO es aceptable tener al micro dentro de esta tarea
		//haciendo nada, esto es solo un ejemplo.
		if(!tiempo_de_diagnostico)
		{
			//vPrintString("Termina el diagnostico\n");
			Board_LED_Set(0, 1);
			vTaskDelay(TIEMPO_DE_DIAGNOSTICO_ms/portTICK_RATE_MS);
			tiempo_de_diagnostico = TIEMPO_DE_DIAGNOSTICO_ms;
			Board_LED_Set(0, 0);
			//vPrintString("Comienza el diagnostico\n");
		}

	}

}

void TeclaEvent (void *pvParameters)
{
	unsigned int EstadoDebounce = NO_OPRIMIDO;
	TECLA tecla;
	portTickType xMeDesperte;

	//Inicio de variables y recuperación de parámetros

	xMeDesperte = xTaskGetTickCount();
	tecla.codigo = BOTON_NO_PRESIONADO;
	tecla.tiempo = 0;

	while(1){
	//debo verificar rebote
	switch(EstadoDebounce){
		case NO_OPRIMIDO:
			vTaskDelayUntil(&xMeDesperte,SCAN_RATE_ms/portTICK_RATE_MS);
			if((unsigned int)Buttons_GetStatus()&((unsigned int)(pvParameters)))	//Si retorna una opresión
				EstadoDebounce = DEBOUNCE;		//Indico que esta corriendo el tiempo Debounce
			break;

		case DEBOUNCE:
			vTaskDelay(TIEMPO_DE_DEBOUNCE_ms/portTICK_RATE_MS);
			EstadoDebounce = VALIDAR;

		case VALIDAR:
			if((unsigned int)Buttons_GetStatus()&(unsigned int)(pvParameters))			//Si retorna algo sigue presionado
			{
				EstadoDebounce=OPRIMIDO;
				tiempo_de_pulsacion = 0;
			}
			else							// fue error
				EstadoDebounce=NO_OPRIMIDO;
			break;

		case OPRIMIDO:
			if(!((unsigned int)Buttons_GetStatus()&(unsigned int)(pvParameters))) //No envio mensaje hasta no soltar
			{
				tecla.codigo = BOTON_PRESIONADO;
				tecla.tiempo = tiempo_de_pulsacion;
				tiempo_de_pulsacion = -1;
				//Envío a la cola, optamos por no bloquear si está llena
				switch((unsigned int)(pvParameters))
				{
					case 0x01:
						xQueueSend( xQueue, &tecla, 0 );
						break;
					case 0x02:
						xQueueSend( xQueue2, &tecla, 0 );
						break;
					case 0x04:
						xQueueSend( xQueue3, &tecla, 0 );
						break;
				}

				EstadoDebounce = NO_OPRIMIDO;
			}
			break;
		default:  break;
		}
	}
}

int main(void) {

	// TODO: insert code here

	//Incialización del Hardware
	initHardware();

	//Trace del RTOS
	//traceSTART();
	xQueue = xQueueCreate(5,sizeof(TECLA));
	xQueue2 = xQueueCreate(5,sizeof(TECLA));
	xQueue3 = xQueueCreate(5,sizeof(TECLA));
	unsigned int vParameter[]={1,2,4};
	//Creación de las tareas
	// xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask )
	xTaskCreate(	JuegoDeLuces, (signed portCHAR* )
					"Luces",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY+2,
					NULL );

	xTaskCreate(	Diagnostico, ( signed portCHAR* )
					"Diag",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY+1,
					NULL );
	xTaskCreate(	TeclaEvent,
					( signed portCHAR* )"Tecl1",
					configMINIMAL_STACK_SIZE,
					(void*)vParameter[0],
					tskIDLE_PRIORITY+2,
					NULL );

	xTaskCreate(	TeclaEvent,
					( signed portCHAR* )"Tecl2",
					configMINIMAL_STACK_SIZE,
					(void*)vParameter[1],
					tskIDLE_PRIORITY+2,
					NULL );

	xTaskCreate(	TeclaEvent,
					( signed portCHAR* )"Tecl3",
					configMINIMAL_STACK_SIZE,
					(void*)vParameter[2],
					tskIDLE_PRIORITY+2,
					NULL );

	//Inicio el Scheduler
	vTaskStartScheduler();

	// Enter an infinite loop, just incrementing a counter
	volatile static int i = 0 ;
	while(1) {
		i++ ;
	}
	return 0 ;
}

void vApplicationTickHook ( void )
{
	if(tiempo_de_diagnostico)
		tiempo_de_diagnostico--;
}


void vApplicationIdleHook ( void )
{
	__ASM volatile ("wfi");
}

