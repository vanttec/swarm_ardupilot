#include <AP_SerialLED/AP_SerialLED.h>
#include "DroneShowLED_SerialLED.h"

DroneShowLED_SerialLED::DroneShowLED_SerialLED(
    DroneShowLED_SerialLEDType type, uint8_t chan, uint8_t num_leds
) : _chan(chan), _type(type), _num_leds(num_leds)
{
}

bool DroneShowLED_SerialLED::init(void)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();
    if (!serialLed) {
        return false;
    }

    if (_type == DroneShowLED_SerialLEDType_NeoPixel) {
        return serialLed->set_num_neopixel(_chan, _num_leds);
    } else if (_type == DroneShowLED_SerialLEDType_ProfiLED) {
        return serialLed->set_num_profiled(_chan, _num_leds);
    }

    return false;
}

void DroneShowLED_SerialLED::set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (true || _last_red != red || _last_green != green || _last_blue != blue) {
        if (try_set_raw_rgb(red, green, blue)) {
            _last_red = red;
            _last_green = green;
            _last_blue = blue;
        }
    }
}

bool DroneShowLED_SerialLED::try_set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();

    if (serialLed) {
        serialLed->set_RGB(_chan, -1, red, green, blue);
        serialLed->send(_chan);
        return true;
    } else {
        return false;
    }
}
