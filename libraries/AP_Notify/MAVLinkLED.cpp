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

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>     // because of CHECK_PAYLOAD_SIZE
#include "AP_Notify/AP_Notify.h"
#include "MAVLinkLED.h"

#define MAVLINK_LED_LOW    0x33
#define MAVLINK_LED_MEDIUM 0x7F
#define MAVLINK_LED_HIGH   0xFF
#define MAVLINK_LED_OFF    0x00

MAVLinkLED::MAVLinkLED(uint8_t instance) :
    RGBLed(MAVLINK_LED_OFF, MAVLINK_LED_HIGH, MAVLINK_LED_MEDIUM, MAVLINK_LED_LOW),
    _instance(instance),
    _chan(MAVLINK_COMM_0)
{
}

bool MAVLinkLED::init()
{
    GCS_MAVLINK* gcs_channel = gcs().chan(_instance);

    if (gcs_channel) {
        _chan = gcs_channel->get_chan();
        return true;
    } else {
        return false;
    }
}

bool MAVLinkLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    mavlink_channel_t chan = _chan;

    if (valid_channel(chan) && mavlink_comm_port[chan] != 0) {
        CHECK_PAYLOAD_SIZE2(DEBUG_VECT);
        mavlink_msg_debug_vect_send(
            _chan, "rgb", AP_HAL::millis(),
            red / 255.0f, green / 255.0f, blue / 255.0f
        );
    }

    return true;
}
