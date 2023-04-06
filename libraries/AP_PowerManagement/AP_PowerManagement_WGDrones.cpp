#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_PowerManagement_WGDrones.h"

MAV_RESULT AP_PowerManagement_WGDrones::handle_preflight_reboot(
    const mavlink_command_long_t &packet
) {
    bool success;

    if (is_equal(packet.param1, 2.0f)) {
        /* Shutdown autopilot (2) */
        success = _send_to_mcu("k");
    } else if (is_equal(packet.param1, 126.0f)) {
        /* Low-power mode (126) */
        success = _send_to_mcu("slp");
    } else if (is_equal(packet.param1, 127.0f)) {
        /* Wake up */
        success = _send_to_mcu("wak");
    } else {
        /* Anything else has to be handled in GCS_MAVLink */
        return MAV_RESULT_UNSUPPORTED;
    }

    return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
}

bool AP_PowerManagement_WGDrones::_send_to_mcu(const char* msg)
{
    get_uart()->printf("$%s@%d,%d\n", msg, mavlink_system.compid, mavlink_system.sysid);
    return true;
}
