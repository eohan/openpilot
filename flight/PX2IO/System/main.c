/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <pios.h>


/* Task Priorities */
#define INIT_TASK_PRIORITY		(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define PROTOCOL_TASK_PRIORITY	(tskIDLE_PRIORITY + 2)
#define FAILSAFE_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

/* Global Variables */

/* Local Variables */
#define INIT_TASK_STACK			(1024 / 4)
#define PROTOCOL_TASK_STACK		(128 / 4)
#define FAILSAFE_TASK_STACK		(128 / 4)
static xTaskHandle initTaskHandle;
static xTaskHandle protocolTaskHandle;
static xTaskHandle failsafeTaskHandle;

/* Function Prototypes */
static void initTask(void *parameters);
static void protocolTask(void *parameters);
static void failsafeTask(void *parameters);

/* Prototype of PIOS_Board_Init() function */
extern void PIOS_Board_Init(void);

int main()
{
	int	result;

	/* initialise the heap */
	vPortInitialiseBlocks();

	/* core PIOS init */
	PIOS_SYS_Init();

	/* create the init task */
	result = xTaskCreate(initTask,
						(const signed char *)"init",
						INIT_TASK_STACK,
						NULL,
						INIT_TASK_PRIORITY,
						&initTaskHandle);
	PIOS_Assert(result == pdPASS);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* either we failed to start the scheduler, or it has returned unexpectedly */
	/* XXX might actually want to reboot here and hope the failure was transient? */
	PIOS_LED_Off(LED1);
	PIOS_LED_On(LED2);
	for(;;) {
		PIOS_LED_Toggle(LED1);
		PIOS_LED_Toggle(LED2);
		PIOS_DELAY_WaitmS(100);
	}

	return 0;
}

/**
 * Initialisation task.
 *
 * Runs board and module initialisation, then terminates.
 */
void
initTask(void *parameters)
{
	/* board driver init */
	PIOS_Board_Init();

	/* Initialize modules */
	MODULE_INITIALISE_ALL;

	/* start tasks */
	xTaskCreate(protocolTask, (const signed char *)"protocol", PROTOCOL_TASK_STACK, NULL, PROTOCOL_TASK_PRIORITY, &protocolTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, protocolTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_PROTOCOL);

	xTaskCreate(failsafeTask, (const signed char *)"failsafe", FAILSAFE_TASK_STACK, NULL, FAILSAFE_TASK_PRIORITY, &failsafeTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, failsafeTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_FAILSAFE);

	/* terminate this task */
	vTaskDelete(NULL);
}


static void
protocolTask(void *parameters)
{
	for (;;) {
		PIOS_LED_Toggle(LED1);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

static void
failsafeTask(void *parameters)
{
	for (;;) {
		PIOS_LED_Toggle(LED2);
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/**
 * @}
 * @}
 */

