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
#include <pios_i2c_slave.h>
#include <protocol.h>

#include "msheap/msheap.h"

/* Task Priorities */
#define INIT_TASK_PRIORITY		(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define PROTOCOL_TASK_PRIORITY	(tskIDLE_PRIORITY + 2)
#define FAILSAFE_TASK_PRIORITY	(tskIDLE_PRIORITY + 3)

/* Global Variables */

/* Local Variables */
#define INIT_TASK_STACK			(1024 / 4)
#define PROTOCOL_TASK_STACK		(512 / 4)
#define FAILSAFE_TASK_STACK		(512 / 4)
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
	PIOS_DELAY_Init();
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
	xTaskCreate(protocolTask,
				(const signed char *)"protocol",
				PROTOCOL_TASK_STACK,
				NULL,
				PROTOCOL_TASK_PRIORITY,
				&protocolTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, protocolTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_PROTOCOL);

	xTaskCreate(failsafeTask,
				(const signed char *)"failsafe",
				FAILSAFE_TASK_STACK,
				NULL,
				FAILSAFE_TASK_PRIORITY,
				&failsafeTaskHandle);
	//TaskMonitorAdd(TASKINFO_RUNNING_PROTOCOL, failsafeTaskHandle);
	//PIOS_WDG_RegisterFlag(PIOS_WDG_FAILSAFE);

	/* terminate this task */
	vTaskDelete(NULL);
}

static int flag = 0;

static void
protocol_callback(uint32_t i2c_id, enum pios_i2c_slave_event event, uint32_t arg)
{
	static struct pios_i2c_slave_txn txns[2];
	static uint8_t		status = 'g';
	static struct iop_command	cmd;

	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "event %d\r\n", event);

	switch (event) {
	case PIOS_I2C_SLAVE_TRANSMIT:
		txns[0].buf = &status;
		txns[0].len = 1;
		PIOS_I2C_SLAVE_Transfer(0, txns, 1);
		status = 'g';
		break;

	case PIOS_I2C_SLAVE_TRANSMIT_DONE:
		// XXX not seeing this when we should
		break;

	case PIOS_I2C_SLAVE_RECEIVE:
		txns[0].buf = (void *)&cmd;
		txns[0].len = sizeof(cmd);
		PIOS_I2C_SLAVE_Transfer(0, txns, 1);
		break;

	case PIOS_I2C_SLAVE_RECEIVE_DONE:
	{
		uint32_t	actual_len;
		int			i;

		/* assume the worst */
		status = 'e';

		/* account for the header size in the transferred length */
		if (arg < 4) {
			PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "short rx %d\r\n", arg);
			break;
		}
		actual_len = arg - 4;

		switch (cmd.opcode) {
		case IOP_SET_PWM:
			if (actual_len < sizeof(struct iop_set_pwm)) {
				PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "short PWM set %d\r\n", arg);
				break;
			}
			for (i = 0; i < IOP_PWM_CHANNELS; i++) {
				PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "servo %d - %u\r\n", i, cmd.d.pwm.values[i]);
				PIOS_Servo_Set(i, cmd.d.pwm.values[i]);
			}
			status = 'g';
			break;

		default:
			PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "unhandled opcode %d\r\n", cmd.opcode);
			// XXX support other commands here
			break;
		}
		break;
	}


	default:
		PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "unhandled event %d\r\n", event);
		break;
	}
}

static void
protocolTask(void *parameters)
{
	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "protocol task start\r\n");
	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "message size %d\r\n", sizeof(struct iop_command));
	PIOS_I2C_Slave_Open(0, protocol_callback);

	for (;;) {
		if (flag) {
			PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "flag\r\n");
			flag = 0;
		}
		PIOS_LED_Toggle(LED1);
		PIOS_LED_Toggle(LED3);

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

static void
failsafeTask(void *parameters)
{
	PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "failsafe task start\r\n");

	for (;;) {
		PIOS_LED_Toggle(LED2);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void
vApplicationTickHook(void)
{
	// XXX this might be a bit drastic later on
	msheap_check();
}

/**
 * @}
 * @}
 */

