/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_MAVLink.h"
#include <AP_HAL/AP_HAL.h>



extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MAVLink::AP_RangeFinder_MAVLink(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    state.last_reading_ms = AP_HAL::millis();
    distance_cm = 0;
	for(uint8_t i=0; i<8; i++) round_distance_cm[i] = 0; // YIG-ADD
}

/*
   detect if a MAVLink rangefinder is connected. We'll detect by
   checking a parameter.
*/
bool AP_RangeFinder_MAVLink::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MAVLink,
    // there is an attached MAVLink rangefinder
    return true;
}

/*
   Set the distance based on a MAVLINK message
*/
void AP_RangeFinder_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(&msg, &packet);

    // only accept distances for downward facing sensors
    //if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_270) // YIG-CHG
	{
        state.last_reading_ms = AP_HAL::millis();
#if 0 // YIG-CHG
        distance_cm = packet.current_distance;
#else
		uint8_t sector = (packet.current_distance / 100) - 1;
		if(sector == 0)
		{
			distance_cm = ((packet.max_distance/100)*100); //Upper
			distance_cm += packet.min_distance/100; //Lower
		}
		else
		{
			round_distance_cm[sector] = ((packet.max_distance/100)*100); //Upper
			round_distance_cm[sector] += packet.min_distance/100; //Lower
		}
#endif
    }
    sensor_type = (MAV_DISTANCE_SENSOR)packet.type;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MAVLink::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_MAVLINK_TIMEOUT_MS) 
	{
        set_status(RangeFinder::RangeFinder_NoData);
        state.distance_cm = 0;
		for(uint8_t i=0; i<8; i++) state.round_distance_cm[i] = 0;
    } else {
        state.distance_cm = distance_cm;
		for(uint8_t i=0; i<8; i++) state.round_distance_cm[i] = round_distance_cm[i];
        update_status();
    }
}
