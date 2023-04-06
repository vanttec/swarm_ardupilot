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

//
/// @file	AP_PowerManagement.h
/// @brief	Support for external power management MCUs that allow the vehicle to
///         be powered off and woken up remotely.
#pragma once

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_PowerManagement_Backend;

class AP_PowerManagement {
public:
    friend class AP_Radio_backend;

    // constructor
    AP_PowerManagement(void);

    // init - initialise power management subsystem
    bool init(void);

    // handle preflight reboot/shutdown requests
    MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet);

    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_PowerManagement *get_singleton(void)
    {
        return _singleton;
    }

    enum AP_PowerManagement_Type {
        PM_TYPE_NONE=0,
        PM_TYPE_WGDRONES=1,
    };

private:
    AP_PowerManagement_Backend *_backend;

    static AP_PowerManagement *_singleton;

    AP_Int8 backend_type;
    AP_Int8 channel;
};
