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
#include "mavlink_telemetry.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"
#include "telemetrysettings.h"

/* Include all UAVObjects that need to be translated to MAVLink */
#include "manualcontrolcommand.h"  /* Remote control / manual commands */
#include "baroaltitude.h"          /* Pressure sensor */
#include "attitudeactual.h"        /* Estimated attitude */
#include "attituderaw.h"           /* Raw attitude sensor measurements */
#include "gpsposition.h"		   /* GPS position */
#include "gpssatellites.h"		   /* GPS satellites */

// Private constants
#define MAX_QUEUE_SIZE   MAVLINK_QUEUE_SIZE
#define STACK_SIZE_BYTES PIOS_MAVLINK_STACK_SIZE
#define TASK_PRIORITY_RX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TX (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_TXPRI (tskIDLE_PRIORITY + 2)
#define REQ_TIMEOUT_MS 250
#define MAX_RETRIES 2
#define STATS_UPDATE_PERIOD_MS 4000
#define CONNECTION_TIMEOUT_MS 8000

// Private types

// Private variables
static uint32_t telemetryPort; // FIXME INIT
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

// Private functions
static void telemetryTxTask(void *parameters);
static void telemetryRxTask(void *parameters);
//static int32_t transmitData(uint8_t * data, int32_t length);
static void registerObject(UAVObjHandle obj);
static void updateObject(UAVObjHandle obj);
static int32_t addObject(UAVObjHandle obj);
static int32_t setUpdatePeriod(UAVObjHandle obj, int32_t updatePeriodMs);
static void processObjEvent(UAVObjEvent * ev);
static void updateTelemetryStats();
static void gcsTelemetryStatsUpdated();
static void updateSettings();

#include "mavlink_types.h"

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
mavlink_system_t mavlink_system;
static mavlink_message_t rx_msg;
//static mavlink_message_t tx_msg;
static mavlink_status_t rx_status;
static uint8_t mavlinkTxBuf[MAVLINK_MAX_PACKET_LEN];

///**
// * @brief Send one char (uint8_t) over a comm channel
// *
// * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
// * @param ch Character to send
// */
//static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
//{
//    if (chan == MAVLINK_COMM_0)
//    {
//    	uint32_t outputPort;
//
//    	// Determine input port (USB takes priority over telemetry port)
//    #if defined(PIOS_INCLUDE_USB_HID)
//    	if (PIOS_USB_HID_CheckAvailable(0)) {
//    		outputPort = PIOS_COM_TELEM_USB;
//    	} else
//    #endif /* PIOS_INCLUDE_USB_HID */
//    	{
//    		outputPort = telemetryPort;
//    	}
//
//    	PIOS_COM_SendBufferNonBlocking(outputPort, &ch, 1);
//    }
//    if (chan == MAVLINK_COMM_1)
//    {
//    	PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_USB, &ch, 1);
//    }
//}

#include "common/mavlink.h"

/* 3: Define waypoint helper functions */
void mavlink_wpm_send_message(mavlink_message_t* msg);
void mavlink_wpm_send_gcs_string(const char* string);
uint64_t mavlink_wpm_get_system_timestamp();

/* 4: Include waypoint protocol */
#include <waypoints.h>
mavlink_wpm_storage wpm;

/* Provide the interface functions for the waypoint manager */

/*
 *  @brief Sends a MAVLink message over UDP
 */
void mavlink_wpm_send_message(mavlink_message_t* msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, msg);
	// Send buffer
	PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
}

void mavlink_wpm_send_gcs_string(const char* string)
{
	const int len = 50;
	mavlink_statustext_t status;
	char* str = (char*)status.text;
	int i = 0;
	while (i < len - 1)
	{
		str[i] = string[i];
		if (string[i] == '\0')
			break;
		i++;
	}
	str[i] = '\0'; // Enforce null termination
	mavlink_message_t msg;

	mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &status);
}

uint64_t mavlink_wpm_get_system_timestamp()
{
//	struct timeval tv;
//	gettimeofday(&tv, NULL);
//	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return xTaskGetTickCount() * portTICK_RATE_MS * 1000;
}



/**
 * Initialise the telemetry module
 * \return -1 if initialisation failed
 * \return 0 on success
 */
int32_t MAVLinkStart(void)
{

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
int32_t MAVLinkInitialize(void)
{
	UAVObjEvent ev;

	// Initialize vars
	timeOfLastObjectUpdate = 0;

	// Create object queues
	queue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#if defined(PIOS_TELEM_PRIORITY_QUEUE)
	priorityQueue = xQueueCreate(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
#endif
	
	// Get telemetry settings object
	updateSettings();

	// Initialize waypoint protocol
	mavlink_wpm_init(&wpm);

	// Initialize parameter protocol

	// Process all registered objects and connect queue for updates
	UAVObjIterate(&registerObject);

	// Create periodic event that will be used to update the telemetry stats
	txErrors = 0;
	txRetries = 0;
	memset(&ev, 0, sizeof(UAVObjEvent));
	EventPeriodicQueueCreate(&ev, priorityQueue, STATS_UPDATE_PERIOD_MS);

	// Listen to objects of interest
	GCSTelemetryStatsConnectQueue(priorityQueue);
	TelemetrySettingsConnectQueue(priorityQueue);

	return 0;
}

MODULE_INITCALL(MAVLinkInitialize, MAVLinkStart)

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

// FIXME XXX

#include "systemstats.h"
#include "flighttelemetrystats.h"
#include "gcstelemetrystats.h"

FlightTelemetryStatsData flightStats;
GCSTelemetryStatsData gcsTelemetryStatsData;
static AttitudeActualData attitudeActual;
static AttitudeRawData attitudeRaw;
static BaroAltitudeData baroAltitude;
static GPSPositionData gpsPosition;
//static ManualControlCommandData manualControl;
static mavlink_raw_imu_t attitude_raw;
static mavlink_attitude_t attitude;
static mavlink_scaled_pressure_t pressure;
static mavlink_gps_raw_int_t gps_raw;
//static mavlink_rc_channels_raw_t rc_channels;
//static mavlink_debug_vect_t debug;
static uint32_t lastOperatorHeartbeat = 0;


/**
 * Processes queue events
 */
static void processObjEvent(UAVObjEvent * ev)
{
//	UAVObjMetadata metadata;
//	FlightTelemetryStatsData flightStats;
//	GCSTelemetryStatsData gcsTelemetryStatsData;
//	int32_t retries;
//	int32_t success;

	AlarmsClear(SYSTEMALARMS_ALARM_TELEMETRY);

	if (ev->obj == 0) {
		updateTelemetryStats();
	} else if (ev->obj == GCSTelemetryStatsHandle()) {
		gcsTelemetryStatsUpdated();
	} else if (ev->obj == TelemetrySettingsHandle()) {
		updateSettings();
	} else {
		mavlink_message_t msg;

		mavlink_system.sysid = 20;
		mavlink_system.compid = MAV_COMP_ID_IMU;
		mavlink_system.type = MAV_FIXED_WING;
		uint8_t mavClass = MAV_CLASS_OPENPILOT;


		uint32_t objId;

		// Setup type and object id fields
		objId = UAVObjGetID(ev->obj);

//		uint64_t timeStamp = 0;
		switch(objId) {
			case ATTITUDEACTUAL_OBJID:
			{
				AttitudeActualGet(&attitudeActual);
				AttitudeRawGet(&attitudeRaw);

				// Copy data
				attitude_raw.xacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_X];
				attitude_raw.yacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Y];
				attitude_raw.zacc = attitudeRaw.accels[ATTITUDERAW_ACCELS_Z];
				attitude_raw.xgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_X];
				attitude_raw.ygyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Y];
				attitude_raw.zgyro = attitudeRaw.gyros[ATTITUDERAW_GYROS_Z];
				attitude_raw.xmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_X];
				attitude_raw.ymag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Y];
				attitude_raw.zmag = attitudeRaw.magnetometers[ATTITUDERAW_MAGNETOMETERS_Z];

				mavlink_msg_raw_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &attitude_raw);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);

				attitude.roll  = (attitudeActual.Roll/180.0f)*3.14159265f;
				attitude.pitch = (attitudeActual.Pitch/180.0f)*3.14159265f;
				attitude.yaw   = (attitudeActual.Yaw/180.0f)*3.14159265f;

				attitude.rollspeed  = (attitudeActual.RollSpeed/180.0f)*3.14159265f;
				attitude.pitchspeed = (attitudeActual.PitchSpeed/180.0f)*3.14159265f;
				attitude.yawspeed   = (attitudeActual.YawSpeed/180.0f)*3.14159265f;

				mavlink_msg_attitude_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &attitude);
				// Copy the message to the send buffer
				len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
//
//				mavlink_msg_attitude_send(MAVLINK_COMM_0, timeStamp,attitudeActual.Roll,
//						attitudeActual.Pitch,attitudeActual.Yaw,
//						attitudeRaw.gyros[ATTITUDERAW_GYROS_X],
				//						attitudeRaw.gyros[ATTITUDERAW_GYROS_Y],
				//						attitudeRaw.gyros[ATTITUDERAW_GYROS_Z]);
				break;
			}
			case BAROALTITUDE_OBJID:
			{
				BaroAltitudeGet(&baroAltitude);
				pressure.press_abs = baroAltitude.Pressure*10.0f;
				pressure.temperature = baroAltitude.Temperature*100.0f;
				mavlink_msg_scaled_pressure_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &pressure);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
				break;
			}
			case FLIGHTTELEMETRYSTATS_OBJID:
			{
//				FlightTelemetryStatsData flightTelemetryStats;
				FlightTelemetryStatsGet(&flightStats);

				// XXX this is a hack to make it think it got a confirmed
				// connection
				flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
				GCSTelemetryStatsGet(&gcsTelemetryStatsData);
				gcsTelemetryStatsData.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
//
//
//				//mavlink_msg_heartbeat_send(MAVLINK_COMM_0,mavlink_system.type,mavClass);
//				mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, mavClass);
//				// Copy the message to the send buffer
//				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
//				// Send buffer
//				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
				break;
			}
			case SYSTEMSTATS_OBJID:
			{
				//mavlink_msg_heartbeat_send(MAVLINK_COMM_0,mavlink_system.type,mavClass);
				mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, mavClass);
				//mavlink_msg_cpu_load_pack(mavlink_system.sysid, mavlink_system.compid, &msg,ucCpuLoad,ucCpuLoad,0);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);

				uint8_t ucCpuLoad;
				SystemStatsCPULoadGet(&ucCpuLoad);
				uint16_t vbat = 11000;
				mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.mode, mavlink_system.nav_mode, mavlink_system.state, ucCpuLoad*4, vbat, 0, 0);
				//mavlink_msg_debug_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, (float)ucCpuLoad);
				len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
				break;
			}
			case GPSPOSITION_OBJID:
			{
				GPSPositionGet(&gpsPosition);
				gps_raw.lat = gpsPosition.Latitude;
				gps_raw.lon = gpsPosition.Longitude;
				gps_raw.fix_type = gpsPosition.Status;
				mavlink_msg_gps_raw_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &gps_raw);
				// Copy the message to the send buffer
				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
				// Send buffer
				PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);
				break;
			}
			case MANUALCONTROLCOMMAND_OBJID:
			{
//				Manu
//				manualControl.Channel[0] = PIOS_PPM_Get(0);
//				manualControl.Channel[1] = PIOS_PPM_Get(1);
//
//				rc_channels.chan1_raw = manualControl.Channel[0];
//				rc_channels.chan2_raw = manualControl.Channel[1];
//
//				debug.x = PIOS_PPM_Get(0);
//				debug.y = PIOS_PPM_Get(1);
//				debug.z = PIOS_PPM_Get(2);
//				debug.name[0] = 'R';
//				debug.name[1] = 'C';
//				debug.name[2] = 0;
//				debug.usec = 0;
//
//				mavlink_msg_debug_vect_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &debug);



//				mavlink_msg_rc_channels_raw_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &rc_channels);


				// Copy the message to the send buffer
//				uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &msg);
//				// Send buffer
//				PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, mavlinkTxBuf, len);
				break;
			}
			default:
			{
				//printf("unknown object: %x\n",(unsigned int)objId);
				break;
			}
		}







//		if (ev->obj == AttitudeRawHandle()) {
//			// Get object data
//			mavlink_raw_imu_t imu;
//			imu.xacc =
//		}

		// Send buffer
		//transmitData(buf, len);
		//if (outStream!=NULL) (*outStream)(txBuffer, dataOffset+length+CHECKSUM_LENGTH);

//		// Only process event if connected to GCS or if object FlightTelemetryStats is updated
//		FlightTelemetryStatsGet(&flightStats);
//		if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED || ev->obj == FlightTelemetryStatsHandle()) {
//			// Get object metadata
//			UAVObjGetMetadata(ev->obj, &metadata);
//			// Act on event
//			retries = 0;
//			success = -1;
//			if (ev->event == EV_UPDATED || ev->event == EV_UPDATED_MANUAL) {
//				// Send update to GCS (with retries)
//				while (retries < MAX_RETRIES && success == -1) {
//					success = UAVTalkSendObject(ev->obj, ev->instId, metadata.telemetryAcked, REQ_TIMEOUT_MS);	// call blocks until ack is received or timeout
//					++retries;
//				}
//				// Update stats
//				txRetries += (retries - 1);
//				if (success == -1) {
//					++txErrors;
//				}
//			} else if (ev->event == EV_UPDATE_REQ) {
//				// Request object update from GCS (with retries)
//				while (retries < MAX_RETRIES && success == -1) {
//					success = UAVTalkSendObjectRequest(ev->obj, ev->instId, REQ_TIMEOUT_MS);	// call blocks until update is received or timeout
//					++retries;
//				}
//				// Update stats
//				txRetries += (retries - 1);
//				if (success == -1) {
//					++txErrors;
//				}
//			}
//			// If this is a metaobject then make necessary telemetry updates
//			if (UAVObjIsMetaobject(ev->obj)) {
//				updateObject(UAVObjGetLinkedObj(ev->obj));	// linked object will be the actual object the metadata are for
//			}
//		}
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
	int32_t len;

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

		// Block until data are available
		// TODO: Currently we periodically check the buffer for data, update once the PIOS_COM is made blocking
		len = PIOS_COM_ReceiveBufferUsed(inputPort);
		for (int32_t n = 0; n < len; ++n) {
//			PIOS_LED_On(LED2);
//			PIOS_LED_On(LED1);
			if (mavlink_parse_char(MAVLINK_COMM_0, PIOS_COM_ReceiveBuffer(inputPort), &rx_msg, &rx_status))
			{

				// Handle packet with waypoint component
				mavlink_wpm_message_handler(&rx_msg);

				switch (rx_msg.msgid)
				{
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				lastOperatorHeartbeat = xTaskGetTickCount() * portTICK_RATE_MS;
				break;
			}
				case MAVLINK_MSG_ID_SET_MODE:
				{
					mavlink_set_mode_t mode;
					mavlink_msg_set_mode_decode(&rx_msg, &mode);
					// Check if this system should change the mode
					if (mode.target == mavlink_system.sysid)
					{
						//sys_set_mode(mode.mode);

						mavlink_system.mode = mode.mode;
//						mavlink_system.nav_mode = MAV_NAV_LOST;
//						mavlink_system.mode = mode.mode;
//						mavlink_system.state = MAV_STATE_ACTIVE;
//						uint16_t vbat = 11000;
//
//						// Emit current mode
//						mavlink_msg_sys_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, MAVLINK_COMM_0, &tx_msg, mavlink_system.mode, mavlink_system.nav_mode,
//								mavlink_system.state, 0,vbat, 0, 0);
//						// Send message
//						uint16_t len = mavlink_msg_to_send_buffer(mavlinkTxBuf, &tx_msg);
//						// Send buffer
//						PIOS_COM_SendBufferNonBlocking(telemetryPort, mavlinkTxBuf, len);

					}
				}
				break;
				case MAVLINK_MSG_ID_ACTION:
				{
					//									execute_action(mavlink_msg_action_get_action(msg));
					//
					//									//Forwart actions from Xbee to Onboard Computer and vice versa
					//									if (chan == MAVLINK_COMM_1)
					//									{
					//										mavlink_send_uart(MAVLINK_COMM_0, msg);
					//									}
					//									else if (chan == MAVLINK_COMM_0)
					//									{
					//										mavlink_send_uart(MAVLINK_COMM_1, msg);
					//									}
				}
				break;
				}

			}
		}
		vTaskDelay(5);	// <- remove when blocking calls are implemented

	}
}

///**
// * Transmit data buffer to the modem or USB port.
// * \param[in] data Data buffer to send
// * \param[in] length Length of buffer
// * \return 0 Success
// */
//static int32_t transmitData(uint8_t * data, int32_t length)
//{
//	uint32_t outputPort;
//
//	// Determine input port (USB takes priority over telemetry port)
//#if defined(PIOS_INCLUDE_USB_HID)
//	if (PIOS_USB_HID_CheckAvailable(0)) {
//		outputPort = PIOS_COM_TELEM_USB;
//	} else
//#endif /* PIOS_INCLUDE_USB_HID */
//	{
//		outputPort = telemetryPort;
//	}
//
//	return PIOS_COM_SendBufferNonBlocking(outputPort, data, length);
//}

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
//	flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
//	gcsStats.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;

	FlightTelemetryStatsData flightStats;
	GCSTelemetryStatsData gcsStats;
	uint8_t forceUpdate;
	uint8_t connectionTimeout;
	uint32_t timeNow;

	// Get object data
	FlightTelemetryStatsGet(&flightStats);
	GCSTelemetryStatsGet(&gcsStats);

	// Update stats object
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
		//flightStats.RxDataRate = (float)utalkStats.rxBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		//flightStats.TxDataRate = (float)utalkStats.txBytes / ((float)STATS_UPDATE_PERIOD_MS / 1000.0);
		//flightStats.RxFailures += utalkStats.rxErrors;
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
	if ((timeNow - lastOperatorHeartbeat) > CONNECTION_TIMEOUT_MS) {
		connectionTimeout = 1;
	} else {
		connectionTimeout = 0;
	}

	// Update connection state
	forceUpdate = 1;

	// FIXME Hardcode value for now
//	connectionTimeout = 0;

	// If the GCS heartbeat has been received in the last five seconds
	// set as connected
	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
		if (connectionTimeout == 0) {
			// Switching from disconnected to connected
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
			gcsStats.Status = GCSTELEMETRYSTATS_STATUS_CONNECTED;
		} else {
			// Not switching, no update needed
			forceUpdate = 0;
		}
	} else {
		// We were connected, checking if we're still connected
		if (connectionTimeout == 1) {
			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
			gcsStats.Status = GCSTELEMETRYSTATS_STATUS_DISCONNECTED;
		} else {
			// We were connected and still are, no update needed
			forceUpdate = 0;
		}
	}

	// FIXME HARDCODED VALUES
	flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
	forceUpdate = 1;

//	if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED) {
//		// Wait for connection request
//		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_HANDSHAKEREQ) {
//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK;
//		}
//	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_HANDSHAKEACK) {
//		// Wait for connection
//		if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_CONNECTED) {
//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_CONNECTED;
//		} else if (gcsStats.Status == GCSTELEMETRYSTATS_STATUS_DISCONNECTED) {
//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
//		}
//	} else if (flightStats.Status == FLIGHTTELEMETRYSTATS_STATUS_CONNECTED) {
//		if (gcsStats.Status != GCSTELEMETRYSTATS_STATUS_CONNECTED || connectionTimeout) {
//			flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
//		} else {
//			forceUpdate = 0;
//		}
//	} else {
//		flightStats.Status = FLIGHTTELEMETRYSTATS_STATUS_DISCONNECTED;
//	}

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

    // Set port speed
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_2400) PIOS_COM_ChangeBaud(telemetryPort, 2400);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_4800) PIOS_COM_ChangeBaud(telemetryPort, 4800);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_9600) PIOS_COM_ChangeBaud(telemetryPort, 9600);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_19200) PIOS_COM_ChangeBaud(telemetryPort, 19200);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_38400) PIOS_COM_ChangeBaud(telemetryPort, 38400);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_57600) PIOS_COM_ChangeBaud(telemetryPort, 57600);
    else
    if (settings.Speed == TELEMETRYSETTINGS_SPEED_115200) PIOS_COM_ChangeBaud(telemetryPort, 115200);
}

/**
  * @}
  * @}
  */
