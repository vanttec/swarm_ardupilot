#pragma once

/// @file   AC_DroneShowLED.h
/// @brief  Abstract LED that is used as a color channel output in a drone show

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class DroneShowLED
{
private:
    /**
     * Current gamma correction exponent of the LED.
     */
    float _gamma;

    /**
     * Gamma correction lookup table that maps uncorrected RGB components to
     * gamma-corrected values.
     */
    uint8_t _gamma_lookup_table[256];

public:
    DroneShowLED() : _gamma(0.0f) {
        set_gamma(1.0f);
    };
    virtual ~DroneShowLED() {};

    /**
     * Initializes the LED.
     */
    virtual bool init() { return true; };

    /**
     * Sets the gamma correction exponent of the LED.
     */
    void set_gamma(float value) {
        if (is_equal(value, _gamma)) {
            return;
        }

        _gamma = value;

        _update_gamma_lookup_table();
    }

    /**
     * Sets the color of the LED.
     * 
     * Red, green and blue channel values used in this function are _before_
     * gamma correction. The function will apply gamma correction on its own.
     */
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
        set_raw_rgb(
            _gamma_lookup_table[red],
            _gamma_lookup_table[green],
            _gamma_lookup_table[blue]
        );
    }

protected:
    /**
     * Sets the raw color of the LED.
     * 
     * Red, green and blue channel values used in this function are _after_
     * gamma correction. The function will simply forward them to the appropriate
     * output device.
     */
    virtual void set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

private:
    /**
     * Updates the gamma correction lookup table when the gamma exponent changes.
     */
    void _update_gamma_lookup_table();
};
