#pragma once

/// @file   AC_DroneShowLED_SerialLED.h
/// @brief  Drone show LED that sends its output to a NeoPixel LED strip.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"

typedef enum {
    DroneShowLED_SerialLEDType_NeoPixel,
    DroneShowLED_SerialLEDType_ProfiLED
} DroneShowLED_SerialLEDType;

/**
 * RGB LED implementation that sends the current state of the RGB LED to a
 * NeoPixel LED strip.
 */
class DroneShowLED_SerialLED: public DroneShowLED {
public:
    DroneShowLED_SerialLED(
        DroneShowLED_SerialLEDType type, uint8_t chan = 1, uint8_t num_leds = 16
    );

    /* Do not allow copies */
    DroneShowLED_SerialLED(const DroneShowLED_SerialLED &other) = delete;
    DroneShowLED_SerialLED &operator=(const DroneShowLED_SerialLED&) = delete;

protected:
    bool init(void) override;
    void set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    uint8_t _chan;
    uint8_t _num_leds;
    DroneShowLED_SerialLEDType _type;

    uint8_t _last_red, _last_green, _last_blue;

    bool try_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
};
