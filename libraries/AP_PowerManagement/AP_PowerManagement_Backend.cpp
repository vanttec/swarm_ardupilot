#include <AP_HAL/AP_HAL.h>

#include "AP_PowerManagement.h"
#include "AP_PowerManagement_Backend.h"

extern const AP_HAL::HAL& hal;

AP_PowerManagement_Backend::AP_PowerManagement_Backend(AP_PowerManagement& frontend)
    : _power_mgmt(frontend)
{
}
