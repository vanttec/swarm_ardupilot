#include <AP_HAL/AP_HAL.h>

#include "AP_PowerManagement.h"
#include "AP_PowerManagement_Backend.h"
#include "AP_PowerManagement_WGDrones.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_PowerManagement::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Set type of power management module
    // @Description: The type of power management module on the vehicle.
    // @Values: 0:None,1:WGDrones
    // @User: Advanced
    AP_GROUPINFO("TYPE",  1, AP_PowerManagement, backend_type, PM_TYPE_NONE),

    // @Param: CHAN
    // @DisplayName: Set channel index of power management module
    // @Description: The channel index of power management module on the vehicle.
    // @User: Advanced
    AP_GROUPINFO("CHAN",  2, AP_PowerManagement, channel, 0),

    AP_GROUPEND
};

AP_PowerManagement *AP_PowerManagement::_singleton;

// constructor
AP_PowerManagement::AP_PowerManagement(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("Multiple AP_PowerManagement declarations");
    }
    _singleton = this;
}

bool AP_PowerManagement::init(void)
{
    switch (backend_type) {
    case PM_TYPE_WGDRONES:
        _backend = new AP_PowerManagement_WGDrones(*this, channel);
        break;
    default:
        break;
    }
    
    if (!_backend) {
        return false;
    }

    return _backend->init();
}

MAV_RESULT AP_PowerManagement::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    if (!_backend) {
        return MAV_RESULT_UNSUPPORTED;
    }
    return _backend->handle_preflight_reboot(packet);
}
