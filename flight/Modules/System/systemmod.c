/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @brief The OpenPilot Modules do the majority of the control in OpenPilot.  The 
 * @ref SystemModule "System Module" starts all the other modules that then take care
 * of all the telemetry and control algorithms and such.  This is done through the @ref PIOS 
 * "PIOS Hardware abstraction layer" which then contains hardware specific implementations
 * (currently only STM32 supported)
 *
 * @{ 
 * @addtogroup SystemModule System Module
 * @brief Initializes PIOS and other modules runs monitoring
 * After initializing all the modules (currently selected by Makefile but in
 * future controlled by configuration on SD card) runs basic monitoring and
 * alarms.
 * @{ 
 *
 * @file       systemmod.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      System module
 *
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

#include "openpilot.h"
#include "systemmod.h"
#include "objectpersistence.h"
#include "flightstatus.h"
#include "systemstats.h"
#include "systemsettings.h"
#include "i2cstats.h"
#include "taskinfo.h"
#include "watchdogstatus.h"
#include "taskmonitor.h"
#include "pios_config.h"


// Private constants
#define SYSTEM_UPDATE_PERIOD_MS 1000
#define LED_BLINK_RATE_HZ 5

#ifndef IDLE_COUNTS_PER_SEC_AT_NO_LOAD
#ifdef STM32F2XX
#define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 509528 // STM32F2xx, calibrated by running tests/test_cpuload.c
#endif
#ifdef STM32F4XX
#define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 1467993  // STM32F4xx, calibrated by running tests/test_cpuload.c
#endif
#if !(defined(STM32F2XX)) && !(defined(STM32F4XX))
#define IDLE_COUNTS_PER_SEC_AT_NO_LOAD 1172265	// STM32F1xx, calibrated by running tests/test_cpuload.c
											  // must be updated if the FreeRTOS or compiler
											  // optimisation options are changed.
#endif
#endif

#if defined(PIOS_SYSTEM_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_SYSTEM_STACK_SIZE
#else
#define STACK_SIZE_BYTES 2048
#endif

#define TASK_PRIORITY (tskIDLE_PRIORITY+2)

// Private types

// Private variables
static uint32_t idleCounter;
static uint32_t idleCounterClear;
static xTaskHandle systemTaskHandle;
static bool stackOverflow;
static bool mallocFailed;

// Private functions
static void objectUpdatedCb(UAVObjEvent * ev);
static void updateStats();
static void updateSystemAlarms();
static void systemTask(void *parameters);
#if defined(DIAGNOSTICS)
static void updateI2Cstats();
static void updateWDGstats();
#endif
/**
 * Create the module task.
 * \returns 0 on success or -1 if initialization failed
 */
int32_t SystemModStart(void)
{
	// Initialize vars
	stackOverflow = false;
	mallocFailed = false;
	// Create system task
	xTaskCreate(systemTask, (signed char *)"System", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &systemTaskHandle);
	// Register task
	TaskMonitorAdd(TASKINFO_RUNNING_SYSTEM, systemTaskHandle);

	return 0;
}

/**
 * Initialize the module, called on startup.
 * \returns 0 on success or -1 if initialization failed
 */
int32_t SystemModInitialize(void)
{

	// Must registers objects here for system thread because ObjectManager started in OpenPilotInit
	SystemSettingsInitialize();
	SystemStatsInitialize();
	ObjectPersistenceInitialize();
#if defined(DIAGNOSTICS)
	TaskInfoInitialize();
	I2CStatsInitialize();
	WatchdogStatusInitialize();
#endif

	SystemModStart();

	return 0;
}

MODULE_INITCALL(SystemModInitialize, 0)
/**
 * System task, periodically executes every SYSTEM_UPDATE_PERIOD_MS
 */
static void systemTask(void *parameters)
{
	portTickType lastSysTime;

	/* create all modules thread */
	MODULE_TASKCREATE_ALL

	// Initialize vars
	idleCounter = 0;
	idleCounterClear = 0;
	lastSysTime = xTaskGetTickCount();

	// Listen for SettingPersistance object updates, connect a callback function
	ObjectPersistenceConnectCallback(&objectUpdatedCb);

	// Main system loop
	while (1) {
		// Update the system statistics
		updateStats();

		// Update the system alarms
		updateSystemAlarms();
#if defined(DIAGNOSTICS)
		updateI2Cstats();
		updateWDGstats();
#endif
		// Update the task status object
		TaskMonitorUpdateAll();

		// Flash the heartbeat LED
		PIOS_LED_Toggle(LED1);

		// Turn on the error LED if an alarm is set
#if (PIOS_LED_NUM > 1)
		if (AlarmsHasErrors()) {
			PIOS_LED_Toggle(LED2);
		} else if (AlarmsHasWarnings())	{
			PIOS_LED_On(LED2);
		} else {
			PIOS_LED_Off(LED2);
		}
#endif

		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);

		// Wait until next period
		if(flightStatus.Armed == FLIGHTSTATUS_ARMED_ARMED) {
			vTaskDelayUntil(&lastSysTime, SYSTEM_UPDATE_PERIOD_MS / portTICK_RATE_MS / (LED_BLINK_RATE_HZ * 2) );
		} else {
			vTaskDelayUntil(&lastSysTime, SYSTEM_UPDATE_PERIOD_MS / portTICK_RATE_MS);
		}
	}
}

/**
 * Function called in response to object updates
 */
static void objectUpdatedCb(UAVObjEvent * ev)
{
	ObjectPersistenceData objper;
	UAVObjHandle obj;

//	// DEBUG
//	if (ev->obj == SystemAlarmsHandle()) {
//		SystemAlarmsData alarm;
//		SystemAlarmsGet(&alarm);
//		PIOS_COM_SendFormattedString(PIOS_COM_DEBUG, "ALARM: %d\r\n", alarm.Alarm);
//	}

	// If the object updated was the ObjectPersistence execute requested action
	if (ev->obj == ObjectPersistenceHandle()) {
		// Get object data
		ObjectPersistenceGet(&objper);

		int retval = -1;
		// Execute action
		if (objper.Operation == OBJECTPERSISTENCE_OPERATION_LOAD) {
			if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
				// Get selected object
				obj = UAVObjGetByID(objper.ObjectID);
				if (obj == 0) {
					return;
				}
				// Load selected instance
				retval = UAVObjLoad(obj, objper.InstanceID);
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjLoadSettings();
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjLoadMetaobjects();
			}
		} else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_SAVE) {
			if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
				// Get selected object
				obj = UAVObjGetByID(objper.ObjectID);
				if (obj == 0) {
					return;
				}
				// Save selected instance
				retval = UAVObjSave(obj, objper.InstanceID);
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjSaveSettings();
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjSaveMetaobjects();
			}
		} else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_DELETE) {
			if (objper.Selection == OBJECTPERSISTENCE_SELECTION_SINGLEOBJECT) {
				// Get selected object
				obj = UAVObjGetByID(objper.ObjectID);
				if (obj == 0) {
					return;
				}
				// Delete selected instance
				retval = UAVObjDelete(obj, objper.InstanceID);
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLSETTINGS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjDeleteSettings();
			} else if (objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLMETAOBJECTS
				   || objper.Selection == OBJECTPERSISTENCE_SELECTION_ALLOBJECTS) {
				retval = UAVObjDeleteMetaobjects();
			}
		} else if (objper.Operation == OBJECTPERSISTENCE_OPERATION_FULLERASE) {
			retval = -1;
#if defined(PIOS_INCLUDE_FLASH_SECTOR_SETTINGS)
			retval = PIOS_FLASHFS_Format();
#endif
		}
		if(retval == 0) { 
			objper.Operation = OBJECTPERSISTENCE_OPERATION_COMPLETED;
			ObjectPersistenceSet(&objper);
		}
	}
}

/**
 * Called periodically to update the I2C statistics 
 */
#if defined(DIAGNOSTICS)
static void updateI2Cstats() 
{
#if defined(PIOS_INCLUDE_I2C)
	I2CStatsData i2cStats;
	I2CStatsGet(&i2cStats);
	
	struct pios_i2c_fault_history history;
	PIOS_I2C_GetDiagnostics(&history, &i2cStats.event_errors);
	
	for(uint8_t i = 0; (i < I2C_LOG_DEPTH) && (i < I2CSTATS_EVENT_LOG_NUMELEM); i++) {
		i2cStats.evirq_log[i] = history.evirq[i];
		i2cStats.erirq_log[i] = history.erirq[i];
		i2cStats.event_log[i] = history.event[i];
		i2cStats.state_log[i] = history.state[i];		
	}
	i2cStats.last_error_type = history.type;
	I2CStatsSet(&i2cStats);
#endif
}

static void updateWDGstats() 
{
	WatchdogStatusData watchdogStatus;
	watchdogStatus.BootupFlags = PIOS_WDG_GetBootupFlags();
	watchdogStatus.ActiveFlags = PIOS_WDG_GetActiveFlags();
	WatchdogStatusSet(&watchdogStatus);
}
#endif


/**
 * Called periodically to update the system stats
 */
static uint16_t GetFreeIrqStackSize(void)
{
	uint32_t i = 0x200;

#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK)
extern uint32_t _irq_stack_top;
extern uint32_t _irq_stack_end;
uint32_t pattern = 0x0000A5A5;
uint32_t *ptr = &_irq_stack_end;

#if 1 /* the ugly way accurate but takes more time, useful for debugging */
	uint32_t stack_size = (((uint32_t)&_irq_stack_top - (uint32_t)&_irq_stack_end) & ~3 ) / 4;

	for (i=0; i< stack_size; i++)
	{
		if (ptr[i] != pattern)
		{
			i=i*4;
			break;
		}
	}
#else /* faster way but not accurate */
	if (*(volatile uint32_t *)((uint32_t)ptr + IRQSTACK_LIMIT_CRITICAL) != pattern)
	{
		i = IRQSTACK_LIMIT_CRITICAL - 1;
	}
	else if (*(volatile uint32_t *)((uint32_t)ptr + IRQSTACK_LIMIT_WARNING) != pattern)
	{
		i = IRQSTACK_LIMIT_WARNING - 1;
	}
	else
	{
		i = IRQSTACK_LIMIT_WARNING;
	}
#endif
#endif
	return i;
}

/**
 * Called periodically to update the system stats
 */
static void updateStats()
{
	static portTickType lastTickCount = 0;
	SystemStatsData stats;

	// Get stats and update
	SystemStatsGet(&stats);
	stats.FlightTime = xTaskGetTickCount() * portTICK_RATE_MS;
#if defined(ARCH_POSIX) || defined(ARCH_WIN32)
	// POSIX port of FreeRTOS doesn't have xPortGetFreeHeapSize()
	stats.HeapRemaining = 10240;
#else
	stats.HeapRemaining = xPortGetFreeHeapSize();
#endif

	// Get Irq stack status
	stats.IRQStackRemaining = GetFreeIrqStackSize();

	// When idleCounterClear was not reset by the idle-task, it means the idle-task did not run
	if (idleCounterClear) {
		idleCounter = 0;
	}

	portTickType now = xTaskGetTickCount();
	if (now > lastTickCount) {
		uint32_t dT = (xTaskGetTickCount() - lastTickCount) * portTICK_RATE_MS;	// in ms
		stats.CPULoad =
			100 - (uint8_t) roundf(100.0f * ((float)idleCounter / ((float)dT / 1000.0f)) / (float)IDLE_COUNTS_PER_SEC_AT_NO_LOAD);
	} //else: TickCount has wrapped, do not calc now
	lastTickCount = now;
	idleCounterClear = 1;
	
#if defined(PIOS_INCLUDE_ADC) && defined(PIOS_ADC_USE_TEMP_SENSOR)
#if defined(STM32F2XX) || defined(STM32F4XX)
	// STM32F2XX and STM32F4XX sensor values
	float temp_voltage = 3.3f * PIOS_ADC_PinGet(5) / ((1 << 12) - 1);
	const float STM32_TEMP_V25 = 0.76f; /* V */
	const float STM32_TEMP_AVG_SLOPE = 2.5f; /* mV/C */
	stats.CPUTemp = (temp_voltage-STM32_TEMP_V25) * 1000.0f / STM32_TEMP_AVG_SLOPE + 25.0f;
#else
	float temp_voltage = 3.3f * PIOS_ADC_PinGet(0) / ((1 << 12) - 1);
	const float STM32_TEMP_V25 = 1.43f; /* V */
	const float STM32_TEMP_AVG_SLOPE = 4.3f; /* mV/C */
	stats.CPUTemp = (temp_voltage-STM32_TEMP_V25) * 1000.0f / STM32_TEMP_AVG_SLOPE + 25.0f;
#endif
#endif
	SystemStatsSet(&stats);
}

/**
 * Update system alarms
 */
static void updateSystemAlarms()
{
	SystemStatsData stats;
	UAVObjStats objStats;
	EventStats evStats;
	SystemStatsGet(&stats);

	// Check heap
	if (stats.HeapRemaining < HEAP_LIMIT_CRITICAL) {
		AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_CRITICAL);
	} else if (stats.HeapRemaining < HEAP_LIMIT_WARNING) {
		AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_WARNING);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_OUTOFMEMORY);
	}

#if !defined(ARCH_POSIX) && !defined(ARCH_WIN32) && defined(CHECK_IRQ_STACK)
	// Check IRQ stack
	if (stats.IRQStackRemaining < IRQSTACK_LIMIT_CRITICAL) {
		AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_CRITICAL);
	} else if (stats.IRQStackRemaining < IRQSTACK_LIMIT_WARNING) {
		AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_WARNING);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_OUTOFMEMORY);
	}
#endif

	// Check CPU load
	if (stats.CPULoad > CPULOAD_LIMIT_CRITICAL) {
		AlarmsSet(SYSTEMALARMS_ALARM_CPUOVERLOAD, SYSTEMALARMS_ALARM_CRITICAL);
	} else if (stats.CPULoad > CPULOAD_LIMIT_WARNING) {
		AlarmsSet(SYSTEMALARMS_ALARM_CPUOVERLOAD, SYSTEMALARMS_ALARM_WARNING);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_CPUOVERLOAD);
	}

	// Check for stack overflow
	if (stackOverflow) {
		AlarmsSet(SYSTEMALARMS_ALARM_STACKOVERFLOW, SYSTEMALARMS_ALARM_CRITICAL);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_STACKOVERFLOW);
	}

	// Check for malloc failures
	if (mallocFailed) {
		AlarmsSet(SYSTEMALARMS_ALARM_OUTOFMEMORY, SYSTEMALARMS_ALARM_CRITICAL);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_OUTOFMEMORY);
	}

#if defined(PIOS_INCLUDE_SDCARD)
	// Check for SD card
	if (PIOS_SDCARD_IsMounted() == 0) {
		AlarmsSet(SYSTEMALARMS_ALARM_SDCARD, SYSTEMALARMS_ALARM_ERROR);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_SDCARD);
	}
#endif

	// Check for event errors
	UAVObjGetStats(&objStats);
	EventGetStats(&evStats);
	UAVObjClearStats();
	EventClearStats();
	if (objStats.eventErrors > 0 || evStats.eventErrors > 0) {
		AlarmsSet(SYSTEMALARMS_ALARM_EVENTSYSTEM, SYSTEMALARMS_ALARM_WARNING);
	} else {
		AlarmsClear(SYSTEMALARMS_ALARM_EVENTSYSTEM);
	}
}

/**
 * Called by the RTOS when the CPU is idle, used to measure the CPU idle time.
 */
void vApplicationIdleHook(void)
{
	// Called when the scheduler has no tasks to run
	if (idleCounterClear == 0) {
		++idleCounter;
	} else {
		idleCounter = 0;
		idleCounterClear = 0;
	}
}

/**
 * Called by the RTOS when a stack overflow is detected.
 */
#define DEBUG_STACK_OVERFLOW 0
void vApplicationStackOverflowHook(xTaskHandle * pxTask, signed portCHAR * pcTaskName)
{
	stackOverflow = true;
#if DEBUG_STACK_OVERFLOW
	static volatile bool wait_here = true;
	while(wait_here);
	wait_here = true;
#endif
}

/**
 * Called by the RTOS when a malloc call fails.
 */
#define DEBUG_MALLOC_FAILURES 0
void vApplicationMallocFailedHook(void)
{
	mallocFailed = true;
#if DEBUG_MALLOC_FAILURES
	static volatile bool wait_here = true;
	while(wait_here);
	wait_here = true;
#endif
}

/**
  * @}
  * @}
  */
