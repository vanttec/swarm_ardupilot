#pragma once

/// @file   AC_DroneShowLED_RGB.h
/// @brief  Drone show LED that sends its output to an RGBLed instance.

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Notify/RGBLed.h>
#include "DroneShowLED.h"


class DroneShowLED_RGB : public DroneShowLED {

public:
    /// Constructor. Takes ownership of the LED instance passed into it.
    DroneShowLED_RGB(RGBLed* led) : DroneShowLED(), _rgb_led(led) {};
    ~DroneShowLED_RGB() {};

    /* Do not allow copies */
    DroneShowLED_RGB(const DroneShowLED_RGB &other) = delete;
    DroneShowLED_RGB &operator=(const DroneShowLED_RGB&) = delete;

    bool init() override {
        return _rgb_led->init();
    }

protected:
    void set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue) override {
        _rgb_led->set_rgb(red, green, blue);
    }

private:
    AP_HAL::OwnPtr<RGBLed> _rgb_led;
};
