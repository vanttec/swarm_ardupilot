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
#pragma once

#include "RGBLed.h"
#include <AP_Common/AP_Common.h>

#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED as a
 * DEBUG_VECT message on a MAVLink channel.
 */
class MAVLinkLED: public RGBLed {
public:
    MAVLinkLED(uint8_t instance = 0);
    bool init(void) override;

protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:

    mavlink_channel_t _chan;
    uint8_t _instance;
};
