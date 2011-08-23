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
 * @file       telemetry.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Include file of the telemetry module.
 * 	       As with all modules only the initialize function is exposed all other
 * 	       interactions with the module take place through the event queue and
 *             objects.
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

#include "mavlink_settings_adapter.h"

#include <string.h>

bool getParamByName(const char* name, mavlink_param_union_t* param)
{
	
	// Actuator settings
	int16_t index = getActuatorSettingsParamIndexByName(name);
	if (index > -1)
	{
		getActuatorSettingsParamByIndex(index, param);
		return true;
	}
	
	// No match, return false
	return false;
}

bool setParamByName(const char* name, mavlink_param_union_t* param)
{
	
	// Actuator settings
	int16_t index = getActuatorSettingsParamIndexByName(name);
	if (index > -1)
	{
		setActuatorSettingsParamByIndex(index, param);
		return true;
	}
	
	// No match, return false
	return false;
}

int16_t getActuatorSettingsParamIndexByName(const char* name)
{
	for (int i = 0; i < MAX_ACTUATOR_PARAMS; ++i)
	{
		bool match = true;
		const char* storage_name = getActuatorSettingsParamName(i);
		for (uint8_t j = 0; j < ONBOARD_PARAM_NAME_LENGTH; ++j)
		{
			// Compare
			if (storage_name[j] != name[j])
			{
				match = false;
			}
			
			// End matching if null termination is reached
			if (storage_name[j] == '\0')
			{
				break;
			}
		}
		if (match) return i;
	}
	return -1;
}

static const char* getActuatorSettingsParamNameByIndex(uint16_t index)
{
	switch (index)
	{
		case 0:
			return "fixedwing_roll1";
		case 1:
			return "fixedwing_roll2";
	}
}

bool getActuatorSettingsParamByIndex(uint16_t index, mavlink_param_union_t* param)
{
	ActuatorSettingsData settings;
	ActuatorSettingsGet(&settings);
	switch (index)
	{
		case 0:
			param->param_uint32 = settings.FixedWingRoll1;
			param->type = MAV_DATA_TYPE_UINT32;
			break;
		case 1:
			param->param_uint32 = settings.FixedWingRoll2;
			param->type = MAV_DATA_TYPE_UINT32;
			break;
		default:
			return false;
			break;
	}
	// Not returned in default case, return true
	return true;
}

bool setActuatorSettingsParamByIndex(uint16_t index, mavlink_param_union_t* param)
{
	ActuatorSettingsData settings;
	ActuatorSettingsGet(&settings);
	switch (index)
	{
		case 0:
			settings.FixedWingRoll1 = param->param_uint32;
			break;
		case 1:
			settings.FixedWingRoll2 = param->param_uint32;
			break;
		default:
			return false;
			break;
	}
	
	// Not returned in default case, write and return true
	ActuatorSettingsSet(&settings);
	return true;
}

/**
 * @}
 * @}
 */
