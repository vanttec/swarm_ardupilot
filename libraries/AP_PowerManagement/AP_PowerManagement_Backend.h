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

/*
  Compass driver backend class. Each supported compass sensor type
  needs to have an object derived from this class.
 */
#pragma once

#include "AP_PowerManagement.h"

class AP_PowerManagement;  // forward declaration
class AP_PowerManagement_Backend
{
public:
    AP_PowerManagement_Backend(AP_PowerManagement& frontend);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_PowerManagement_Backend(void) {}

    virtual bool init(void) { return true; }
  
    virtual MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet) {
      return MAV_RESULT_UNSUPPORTED;
    }

protected:
    // access to frontend
    AP_PowerManagement& _power_mgmt;

};
