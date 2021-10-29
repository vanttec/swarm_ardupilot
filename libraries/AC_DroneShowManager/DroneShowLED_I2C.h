#pragma once

/// @file   AC_DroneShowLED_I2C.h
/// @brief  Drone show LED that sends its output to an I2C bus.

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "DroneShowLED.h"


class DroneShowLED_I2C : public DroneShowLED {

public:
    /// Constructor
    DroneShowLED_I2C(uint8_t bus, uint8_t addr);

    /* Do not allow copies */
    DroneShowLED_I2C(const DroneShowLED_I2C &other) = delete;
    DroneShowLED_I2C &operator=(const DroneShowLED_I2C&) = delete;

    bool init(void) override;

protected:
    void set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue) override;

private:
    /** The index of the I2C bus */
    uint8_t _bus;

    /** The 7-bit device address on the I2C bus */
    uint8_t _addr;

    /** I2C device that can be used to write to the I2C bus */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    /** Semaphore that synchronizes access to certain parts of the I2C LED
     * data structure between the main thread and the I2C timer thread */
    HAL_Semaphore _sem;

    /** Stores whether a new command has to be sent on the I2C bus to change the
     * LED state */
    bool _send_required;

    /** Stores the red, green and blue colors to set on the LED */
    uint8_t _last_red, _last_green, _last_blue;

    void _timer(void);
};
