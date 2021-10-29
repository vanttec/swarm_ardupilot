#pragma once

/// @file   AC_DroneShowLED_Debug.h
/// @brief  Drone show LED that sends its output in MAVLink STATUSTEXT messages for debugging purposes.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"
#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED as a
 * STATUSTEXT message on all MAVLink channels.
 */
class DroneShowLED_Debug : public DroneShowLED {
public:
    DroneShowLED_Debug();

protected:
    bool init(void) override;
    void set_raw_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    uint8_t _last_red, _last_green, _last_blue;

    uint8_t packet_overhead() const;
    bool try_set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue);
};
