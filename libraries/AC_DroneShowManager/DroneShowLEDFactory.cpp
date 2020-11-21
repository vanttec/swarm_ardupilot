#include "DroneShowLEDFactory.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/RCOutputRGBLed.h>
#include <AP_Notify/SITL_SFML_LED.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowLED_MAVLink.h"
#include "DroneShowLED_RGB.h"
#include "DroneShowLED_SerialLED.h"

/// Default constructor.
DroneShowLEDFactory::DroneShowLEDFactory()
{
}

DroneShowLED* DroneShowLEDFactory::new_rgb_led_by_type(DroneShowLEDType type, uint8_t num_leds) {
    uint8_t chan_red, chan_green, chan_blue;
    RGBLed* rgb_led = NULL;

    DroneShowLED* result = NULL;

    switch (type) {
        case DroneShowLEDType_MAVLink_Channel0:
            result = new DroneShowLED_MAVLink(MAVLINK_COMM_0);
            break;

        case DroneShowLEDType_MAVLink_Channel1:
            result = new DroneShowLED_MAVLink(MAVLINK_COMM_1);
            break;

        case DroneShowLEDType_MAVLink_Channel2:
            result = new DroneShowLED_MAVLink(MAVLINK_COMM_2);
            break;

        case DroneShowLEDType_MAVLink_Channel3:
            result = new DroneShowLED_MAVLink(MAVLINK_COMM_3);
            break;
            
        case DroneShowLEDType_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(WITH_SITL_RGBLED)
            rgb_led = new SITL_SFML_LED();
#endif
            break;

        case DroneShowLEDType_Servo:
            if (
                SRV_Channels::find_channel(SRV_Channel::k_scripting14, chan_red) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting15, chan_green) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting16, chan_blue)
            ) {
                rgb_led = new RCOutputRGBLed(chan_red, chan_green, chan_blue);
            }
            break;

        case DroneShowLEDType_NeoPixel:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_NeoPixel, 1, num_leds);
            break;

        case DroneShowLEDType_ProfiLED:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_ProfiLED, 1, num_leds);
            break;

        default:
            break;
    }

    if (rgb_led) {
        // Connect the RGB led instance to the notification manager to prevent
        // crashes when RGBLed->set_rgb() tries to access it
        rgb_led->pNotify = &AP::notify();
    }

    if (rgb_led) {
        // Ownership of rgb_led now taken by the DroneShowLED_RGB instance
        result = new DroneShowLED_RGB(rgb_led);
    }

    if (result && !result->init()) {
        // Initialization failed
        delete result;
        result = NULL;
    }

    return result;
}
