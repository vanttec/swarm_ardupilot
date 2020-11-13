#include "DroneShowLEDFactory.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/MAVLinkLED.h>
#include <AP_Notify/RCOutputRGBLed.h>
#include <AP_Notify/SITL_SFML_LED.h>
#include <SRV_Channel/SRV_Channel.h>

/// Default constructor.
DroneShowLEDFactory::DroneShowLEDFactory()
{
}

RGBLed* DroneShowLEDFactory::new_rgb_led_by_type(DroneShowLEDType type) {
    uint8_t chan_red, chan_green, chan_blue;
    RGBLed* result = NULL;

    switch (type) {
        case DroneShowLEDType_MAVLink_Channel0:
            result = new MAVLinkLED(MAVLINK_COMM_0);
            break;

        case DroneShowLEDType_MAVLink_Channel1:
            result = new MAVLinkLED(MAVLINK_COMM_1);
            break;

        case DroneShowLEDType_MAVLink_Channel2:
            result = new MAVLinkLED(MAVLINK_COMM_2);
            break;

        case DroneShowLEDType_MAVLink_Channel3:
            result = new MAVLinkLED(MAVLINK_COMM_3);
            break;
            
        case DroneShowLEDType_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(WITH_SITL_RGBLED)
            result = new SITL_SFML_LED();
#endif
            break;

        case DroneShowLEDType_Servo:
            if (
                SRV_Channels::find_channel(SRV_Channel::k_scripting14, chan_red) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting15, chan_green) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting16, chan_blue)
            ) {
                result = new RCOutputRGBLed(chan_red, chan_green, chan_blue);
            }
            break;

        default:
            break;
    }

    if (result) {
        // Connect the RGB led instance to the notification manager to prevent
        // crashes when RGBLed->set_rgb() tries to access it
        result->pNotify = &AP::notify();
        
        if (!result->init()) {
            // Initialization failed
            delete result;
            result = 0;
        }
    }

    return result;
}
