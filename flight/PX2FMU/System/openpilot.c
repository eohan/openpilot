/**
 ******************************************************************************
 * @addtogroup OpenPilotSystem OpenPilot System
 * @brief These files are the core system files of OpenPilot.
 * They are the ground layer just above PiOS. In practice, OpenPilot actually starts
 * in the main() function of openpilot.c
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @brief This is where the OP firmware starts. Those files also define the compile-time
 * options of the firmware.
 * @{
 * @file       openpilot.c 
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Sets up and runs main OpenPilot tasks.
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
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


/* OpenPilot Includes */
#include "pios.h"
#include "openpilot.h"
#include "uavobjectsinit.h"
#include "systemmod.h"

/* Task Priorities */
#define PRIORITY_TASK_HOOKS             (tskIDLE_PRIORITY + 3)

/* Global Variables */

/* Local Variables */
#define INIT_TASK_PRIORITY	(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define INIT_TASK_STACK		(1024 / 4)										// XXX this seems excessive
static xTaskHandle initTaskHandle;

/* Function Prototypes */
static void initTask(void *parameters);

/* Prototype of generated InitModules() function */
extern void InitModules(void);

/* Prototype of PIOS_Board_Init() function */
extern void PIOS_Board_Init(void);

int
main()
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

	/* If all is well we will never reach here as the scheduler will now be running. */
	/* If we do get here, it will most likely be because we ran out of heap space. */
	PIOS_LED_Off(LED1);
	PIOS_LED_Off(LED2);
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

	/* terminate this task */
	vTaskDelete(NULL);
}

/**
 * @}
 * @}
 */

