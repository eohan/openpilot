/*
 * mavlink_send_bridge.c
 *
 *  Created on: Sep 28, 2011
 *      Author: pixhawk
 */

#include "mavlink_send_bridge.h"

void mavlink_send_uart_bytes(mavlink_channel_t chan, uint8_t* buffer, uint16_t len)
{
    if (chan == MAVLINK_COMM_0)
    {
    	// Determine input port (USB takes priority over telemetry port)
    #if defined(PIOS_INCLUDE_USB_HID)
    	if (PIOS_USB_HID_CheckAvailable(0)) {
    		PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_USB, buffer, len);
    	}
#endif
		PIOS_COM_SendBufferNonBlocking(PIOS_COM_TELEM_RF, buffer, len);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	PIOS_COM_SendBufferNonBlocking(PIOS_COM_AUX, buffer, len);
    }
}
