/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{ 
 * @addtogroup TelemetryModule Telemetry Module
 * @brief Main telemetry module
 * Starts three tasks (RX, TX, and priority TX) that watch event queues
 * and handle all the telemetry of the UAVobjects
 * @{ 
 *
 * @file       telemetry.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Telemetry module, handles telemetry and UAVObject updates
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
#include "telemetry.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"
#include "telemetrysettings.h"

// Private constants
#define MAX_QUEUE_SIZE   TELEM_QUEUE_SIZE
#define STACK_SIZE_BYTES PIOS_TELEM_STACK_SIZE
#define TASK_PRIORITY_RX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TXPRI (tskIDLE_PRIORITY + 2)
#define REQ_TIMEOUT_MS 250
#define MAX_RETRIES 2
#define STATS_UPDATE_PERIOD_MS 4000
#define CONNECTION_TIMEOUT_MS 8000

// Private types

// Private variables
static uint32_t telemetryPort;
static xQueueHandle queue;

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static xQueueHandle priorityQueue;
static xTaskHandle telemetryTxPriTaskHandle;
static void telemetryTxPriTask(void *parameters);
#else
#define priorityQueue queue
#endif

static xTaskHandle telemetryTxTaskHandle;
static xTaskHandle telemetryRxTaskHandle;
static uint32_t txErrors;
static uint32_t txRetries;
static TelemetrySettingsData settings;
static uint32_t timeOfLastObjectUpdate;
static UAVTalkConnection uavTalkCon;

// Private functions
static void telemetryTxTask(void *parameters);
static void telemetryRxTask(void *parameters);
static int32_t transmitData(uint8_t * data, int32_t length);
static void registerObject(UAVObjHandle obj);
static void updateObject(UAVObjHandle obj);
static int32_t addObject(UAVObjHandle obj);
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs);
static void processObjEvent(UAVObjEvent * ev);
static void updateTelemetryStats();
static void gcsTelemetryStatsUpdated();
static void updateSettings();

/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t TelemetryStart(void)
{
	// Process all registered objects and connect queue for updates
	UAVObjIterate(&registerObject);
    
	// Listen to objects of interest
	GCSTelemetryStatsConnectQueue(priorityQueue);
	TelemetrySettingsConnectQueue(priorityQueue);
    
	// Start telemetry tasks
	xTaskCreate(telemetryTxTask, (signed char *)"TelTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TX, &telemetryTxTaskHandle);
	xTaskCreate(telemetryRxTask, (signed char *)"TelRx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_RX, &telemetryRxTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTX, telemetryTxTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYRX, telemetryRxTaskHandle);

#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	xTaskCreate(telemetryTxPriTask, (signed char *)"TelPriTx", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY_TXPRI, &telemetryTxPriTaskHandle);
	TaskMonitorAdd(TASKINFO_RUNNING_TELEMETRYTXPRI, telemetryTxPriTaskHandle);
#endif

	return 0;
}

/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t TelemetryInitialize(void)
{
	UAVObjEvent ev;
    
	FlightTelemetryStatsInitialize();
	GCSTelemetryStatsInitialize();
	TelemetrySettingsInitialize();

	// Initialize vars
	timeOfLastObjectUpdate = 0;

	// Create object queues
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	priorityQueue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#endif

    // Get telemetry settings object
	updateSettings();
    
	// Initialise UAVTalk
	uavTalkCon = UAVTalkInitialize(&transmitData,256);
    
	// Create periodic event that will be used to update the telemetry stats
	txErrors = 0;
	txRetries = 0;
	memset(&ev, 0, sizeof(UAVObjEvent));
	EventPeriodicQueueCreate(&ev, priorityQueue, STATS_UPDATE_PERIOD_MS);
    

	return 0;
}

MODULE_INITCALL(TelemetryInitialize, TelemetryStart)

/**
 * Register a new object, adds object to local list and connects the queue depending on the object's
 * telemetry settings.
 * \param[in] obj Object to connect
 */
static void registerObject(UAVObjHandle obj)
{
	// Setup object for periodic updates
	addObject(obj);

	// Setup object for telemetry updates
	updateObject(obj);
}

/**
 * Update object's queue connections and timer, depending on object's settings
 * \param[in] obj Object to updates
 */
static void updateObject(UAVObjHandle obj)
{
	UAVObjMetadata metadata;
	int32_t eventMask;

	// Get metadata
	UAVObjGetMetadata(obj, &metadata);

	// Setup object depending on update mode
	if (metadata.telemetryUpdateMode == UPDATEMODE_PERIODIC) {
		// Set update period
		setUpdatePeriod(obj, metadata.telemetryUpdatePeriod);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_ONCHANGE) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED | EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_MANUAL) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Connect queue
		eventMask = EV_UPDATED_MANUAL | EV_UPDATE_REQ;
		if (UAVObjIsMetaobject(obj)) {
			eventMask |= EV_UNPACKED;	// we also need to act on remote updates (unpack events)
		}
		UAVObjConnectQueue(obj, priorityQueue, eventMask);
	} else if (metadata.telemetryUpdateMode == UPDATEMODE_NEVER) {
		// Set update period
		setUpdatePeriod(obj, 0);
		// Disconnect queue
		UAVObjDisconnectQueue(obj, priorityQueue);
	}
}

/**
 * Processes queue events
 */
static void processObjEvent(UAVObjEvent * ev)
{
	UAVObjMetadata metadata;
	FlightTelemetryStatsData flightStats;
	int32_t retries;
	int32_t success;

	if (ev->obj == 0) {
		updateTelemetryStats();
	} else if (ev->obj == GCSTelemetryStatsHandle()) {
		gcsTelemetryStatsUpdated();
	} else if (ev->obj == TelemetrySettingsHandle()) {
		updateSettings();
	} else {
		// Only process event if connected to GCS or if object FlightTelemetryStats is updated
		FlightTelemetryStatsGet(&flightStats);
		if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || ev->obj == FlightTelemetryStatsHandle()) {
			// Get object metadata
			UAVObjGetMetadata(ev->obj, &metadata);
			// Act on event
			retries = 0;
			success = -1;
			if (ev->event == EV_UPDATED || ev->event == EV_UPDATED_MANUAL) {
				// Send update to GCS (with retries)
				while (retries < MAX_RETRIES && success == -1) {
					success = UAVTalkSendObject(uavTalkCon, ev->obj, ev->instId, metadata.telemetryAcked, REQ_TIMEOUT_MS);	// call blocks until ack is received or timeout
					++retries;
				}
				// Update stats
				txRetries += (retries - 1);
				if (success == -1) {
					++txErrors;
				}
			} else if (ev->event == EV_UPDATE_REQ) {
				// Request object update from GCS (with retries)
				while (retries < MAX_RETRIES && success == -1) {
					success = UAVTalkSendObjectRequest(uavTalkCon, ev->obj, ev->instId, REQ_TIMEOUT_MS);	// call blocks until update is received or timeout
					++retries;
				}
				// Update stats
				txRetries += (retries - 1);
				if (success == -1) {
					++txErrors;
				}
			}
			// If this is a metaobject then make necessary telemetry updates
			if (UAVObjIsMetaobject(ev->obj)) {
				updateObject(UAVObjGetLinkedObj(ev->obj));	// linked object will be the actual object the metadata are for
			}
		}
	}
}

/**
 * Telemetry transmit task, regular priority
 */
static void telemetryTxTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(queue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}

/**
 * Telemetry transmit task, high priority
 */
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
static void telemetryTxPriTask(void *parameters)
{
	UAVObjEvent ev;

	// Loop forever
	while (1) {
		// Wait for queue message
		if (xQueueReceive(priorityQueue, &ev, portMAX_DELAY) == pdTRUE) {
			// Process event
			processObjEvent(&ev);
		}
	}
}
#endif

/**
 * Telemetry transmit task. Processes queue events and periodic updates.
 */
static void telemetryRxTask(void *parameters)
{
	uint32_t inputPort;

	// Task loop
	while (1) {
#if defined(PIOS_INCLUDE_USB_HID)
		// Determine input port (USB takes priority over telemetry port)
		if (PIOS_USB_HID_CheckAvailable(0)) {
			inputPort = PIOS_COM_TELEM_USB;
		} else
#endif /* PIOS_INCLUDE_USB_HID */
		{
			inputPort = telemetryPort;
		}

		if (inputPort) {
			// Block until data are available
			uint8_t serial_data[1];
			uint16_t bytes_to_process;

			bytes_to_process = PIOS_COM_ReceiveBuffer(inputPort, serial_data, sizeof(serial_data), 500);
			if (bytes_to_process > 0) {
				for (uint8_t i = 0; i < bytes_to_process; i++) {
					UAVTalkProcessInputStream(uavTalkCon,serial_data[i]);
				}
			}
		} else {
			vTaskDelay(5);
		}
	}
}

/**
 * Transmit data buffer to the modem or USB port.
 * \param[in] data Data buffer to send
 * \param[in] length Length of buffer
 * \return 0 Success
 */
static int32_t transmitData(uint8_t * data, int32_t length)
{
	uint32_t outputPort;

	// Determine input port (USB takes priority over telemetry port)
#if defined(PIOS_INCLUDE_USB_HID)
	if (PIOS_USB_HID_CheckAvailable(0)) {
		outputPort = PIOS_COM_TELEM_USB;
	} else
#endif /* PIOS_INCLUDE_USB_HID */
	{
		outputPort = telemetryPort;
	}

	if (outputPort) {
		return PIOS_COM_SendBufferNonBlocking(outputPort, data, length);
	} else {
		return -1;
	}
}

/**
 * Setup object for periodic updates.
 * \param[in] obj The object to update
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t addObject(UAVObjHandle obj)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueCreate(&ev, queue, 0);
}

/**
 * Set update period of object (it must be already setup for periodic updates)
 * \param[in] obj The object to update
 * \param[in] updatePeriodMs The update period in ms, if zero then periodic updates are disabled
 * \return 0 Success
 * \return -1 Failure
 */
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs)
{
	UAVObjEvent ev;

	// Add object for periodic updates
	ev.obj = obj;
	ev.instId = UAVOBJ_ALL_INSTANCES;
	ev.event = EV_UPDATED_MANUAL;
	return EventPeriodicQueueUpdate(&ev, queue, updatePeriodMs);
}

/**
 * Called each time the GCS telemetry stats object is updated.
 * Trigger a flight telemetry stats update if a connection is not
 * yet established.
 */
static void gcsTelemetryStatsUpdated()
{
	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);
	if (flightStats.Status != FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED) {
		updateTelemetryStats();
	}
}

/**
 * Update telemetry statistics and handle connection handshake
 */
static void updateTelemetryStats()
{
	UAVTalkStats utalkStats;
	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	uint8_t forceUpdate;
	uint8_t connectionTimeout;
	uint32_t timeNow;

	// Get stats
	UAVTalkGetStats(uavTalkCon, &utalkStats);
	UAVTalkResetStats(uavTalkCon);

	// Get object data
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);

	// Update stats object
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		flightStats.RxDataRate = (float)utalkStats.rxBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		flightStats.TxDataRate = (float)utalkStats.txBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		flightStats.RxFailures += utalkStats.rxErrors;
		flightStats.TxFailures += txErrors;
		flightStats.TxRetries += txRetries;
		txErrors = 0;
		txRetries = 0;
	} else {
		flightStats.RxDataRate = 0;
		flightStats.TxDataRate = 0;
		flightStats.RxFailures = 0;
		flightStats.TxFailures = 0;
		flightStats.TxRetries = 0;
		txErrors = 0;
		txRetries = 0;
	}

	// Check for connection timeout
	timeNow = xTaskGetTickCount() * portTICK_RATE_MS;
	if (utalkStats.rxObjects > 0) {
		timeOfLastObjectUpdate = timeNow;
	}
	if ((timeNow - timeOfLastObjectUpdate) > CONNECTION_TIMEOUT_MS) {
		connectionTimeout = 1;
	} else {
		connectionTimeout = 0;
	}

	// Update connection state
	forceUpdate = 1;
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
		// Wait for connection request
		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK;
		}
	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK) {
		// Wait for connection
		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_CONNECTED) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
		} else if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_DISCONNECTED) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
		}
	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		if (gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED || connectionTimeout) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
		} else {
			forceUpdate = 0;
		}
	} else {
		flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
	}

	// Update the telemetry alarm
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		AlarmsClear(SYSTEMALARMS_ALARM_TELEMETRY);
	} else {
		AlarmsSet(SYSTEMALARMS_ALARM_TELEMETRY, SYSTEMALARMS_ALARM_ERROR);
	}

	// Update object
	FlightTelemetryStatsSet(&flightStats);

	// Force telemetry update if not connected
	if (forceUpdate) {
		FlightTelemetryStatsUpdated();
	}
}

/**
 * Update the telemetry settings, called on startup and
 * each time the settings object is updated
 */
static void updateSettings()
{
    // Set port
    telemetryPort = PIOS_COM_TELEM_RF;

    // Retrieve settings
    TelemetrySettingsGet(&settings);

    if (telemetryPort) {
	// Set port speed
	if (settings.Speed == TELEMETRYSETTINGS_SPEED_2400) PIOS_COM_ChangeBaud(telemetryPort, 2400);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_4800) PIOS_COM_ChangeBaud(telemetryPort, 4800);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_9600) PIOS_COM_ChangeBaud(telemetryPort, 9600);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_19200) PIOS_COM_ChangeBaud(telemetryPort, 19200);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_38400) PIOS_COM_ChangeBaud(telemetryPort, 38400);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_57600) PIOS_COM_ChangeBaud(telemetryPort, 57600);
	else if (settings.Speed == TELEMETRYSETTINGS_SPEED_115200) PIOS_COM_ChangeBaud(telemetryPort, 115200);
    }
}

// Dummy function for now, should be filled with life
void debug_vect(const char* string, const float x, const float y, const float z)
{

}

void debug_message_buffer(const char* string)
{

}

/**
  * @}
  * @}
  */
