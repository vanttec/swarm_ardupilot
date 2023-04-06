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

/// @file	AP_PowerManagement_WGDrones.h
/// @brief	Power management implementation for WGDrones show drones.
#pragma once

#include "AP_PowerManagement_UART.h"

class AP_PowerManagement_WGDrones : public AP_PowerManagement_UART
{
public:
    AP_PowerManagement_WGDrones(AP_PowerManagement& frontend, uint8_t uart_index) :
        AP_PowerManagement_UART(frontend, uart_index) {}

    virtual MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet) override;

private:
    bool _send_to_mcu(const char* msg);
};
