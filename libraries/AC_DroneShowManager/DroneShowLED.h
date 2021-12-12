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

    /**
     * Last RGB components that were sent on this LED.
     */
    uint8_t _last_red, _last_green, _last_blue;

    /**
     * Number of times we need to repeat LED commands.
     */
    uint8_t _repeat_count;

    /**
     * Number of times we still need to repeat the last LED command.
     */
    uint8_t _repeat_count_left;

public:
    DroneShowLED() :
        _gamma(0.0f), _last_red(0.0f), _last_green(0.0f), _last_blue(0.0f), _repeat_count(0)
    {
        set_gamma(1.0f);
        set_repeat_count(1);
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
        _reset_repeat_count();
    }

    /**
     * Sers the repeat count of the LED, i.e. the number of times a color setting
     * command should be repeated.
     */
    void set_repeat_count(uint8_t value) {
        if (value < 1) {
            value = 1;
        }

        if (value != _repeat_count) {
            _repeat_count = value;
            _reset_repeat_count();
        }
    }

    /**
     * Sets the color of the LED.
     * 
     * Red, green and blue channel values used in this function are _before_
     * gamma correction. The function will apply gamma correction on its own.
     */
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
        if (red != _last_red || green != _last_green || blue != _last_blue) {
            _last_red = red;
            _last_green = green;
            _last_blue = blue;
            _reset_repeat_count();
        }

        repeat_last_command_if_needed();
    }

    /**
     * Repeats the last command to set the color of the RGB LED if needed.
     */
    void repeat_last_command_if_needed() {
        if (_repeat_count_left == 0) {
            return;
        }

        if (set_raw_rgb(
            _gamma_lookup_table[_last_red],
            _gamma_lookup_table[_last_green],
            _gamma_lookup_table[_last_blue]
        )) {
            _repeat_count_left--;
        }
    }

protected:
    /**
     * Sets the raw color of the LED.
     * 
     * Red, green and blue channel values used in this function are _after_
     * gamma correction. The function will simply forward them to the appropriate
     * output device.
     * 
     * Returns whether the new color was set successfully.
     */
    virtual bool set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

private:
    /**
     * Resets the repeat count to its maximum value.
     */
    void _reset_repeat_count() {
        _repeat_count_left = _repeat_count;
    }

    /**
     * Updates the gamma correction lookup table when the gamma exponent changes.
     */
    void _update_gamma_lookup_table();
};
