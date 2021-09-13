#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>     // because of CHECK_PAYLOAD_SIZE2

#include "DroneShowLED_MAVLink.h"

static const char message_name[10] = "rgb";

DroneShowLED_MAVLink::DroneShowLED_MAVLink(uint8_t instance) : DroneShowLED(),
    _instance(instance),
    _chan(MAVLINK_COMM_0),
    _last_red(0),
    _last_green(0),
    _last_blue(0)
{
}

bool DroneShowLED_MAVLink::init()
{
    GCS_MAVLINK* gcs_channel = gcs().chan(_instance);

    if (gcs_channel) {
        _chan = gcs_channel->get_chan();
        return true;
    } else {
        return false;
    }
}

void DroneShowLED_MAVLink::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (_last_red != red || _last_green != green || _last_blue != blue) {
        if (try_set_rgb(red, green, blue)) {
            _last_red = red;
            _last_green = green;
            _last_blue = blue;
        }
    }
}

bool DroneShowLED_MAVLink::try_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    mavlink_channel_t chan = _chan;

    if (valid_channel(chan) && mavlink_comm_port[chan] != 0) {
        CHECK_PAYLOAD_SIZE2(DEBUG_VECT);
        mavlink_msg_debug_vect_send(
            _chan, message_name, AP_HAL::millis(),
            red / 255.0f, green / 255.0f, blue / 255.0f
        );
    }

    return true;
}
