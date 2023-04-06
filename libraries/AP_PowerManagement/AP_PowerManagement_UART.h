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

/// @file	AP_PowerManagement_UART.h
/// @brief	Power management backend base class that sends messsages via a UART.
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_PowerManagement_Backend.h"

/**
 * Power management backend superclass that sends messages via a UART.
 */
class AP_PowerManagement_UART : public AP_PowerManagement_Backend {
public:
    AP_PowerManagement_UART(AP_PowerManagement& frontend, uint8_t uart_index = 0);

protected:
    AP_HAL::UARTDriver* get_uart(void) const;
    
private:
    uint8_t _uart_index;
};
