#pragma once

/// @file   AC_DroneShowLED.h
/// @brief  Abstract LED that is used as a color channel output in a drone show

#include <AP_HAL/AP_HAL.h>

class DroneShowLED
{
public:
    virtual ~DroneShowLED() {};

    /**
     * Initializes the LED.
     */
    virtual bool init() { return true; };

    /**
     * Sets the color of the LED.
     */
    virtual void set_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;
};
