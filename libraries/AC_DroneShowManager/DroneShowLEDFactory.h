#pragma once

/// @file   AC_DroneShowLEDFactory.h
/// @brief  LED factory class that creates LED instances for the drone show manager module

#include "DroneShowLED.h"

// Supported LED types for the drone show manager
enum DroneShowLEDType {
    // No LED light output
    DroneShowLEDType_None = 0,

    // LED light color is sent in a DEBUG_VECT to the MAVLink channel with index 0
    DroneShowLEDType_MAVLink_Channel0 = 1,

    // LED light color is sent in a DEBUG_VECT to the MAVLink channel with index 1
    DroneShowLEDType_MAVLink_Channel1 = 2,

    // LED light color is sent in a DEBUG_VECT to the MAVLink channel with index 2
    DroneShowLEDType_MAVLink_Channel2 = 3,

    // LED light color is sent in a DEBUG_VECT to the MAVLink channel with index 3
    DroneShowLEDType_MAVLink_Channel3 = 4,

    // LED light color is to be forwarded to the SITL simulator
    DroneShowLEDType_SITL = 5,

    // LED light is attached to servo channels
    DroneShowLEDType_Servo = 6,

    // LED light is to be forwarded to a NeoPixel LED strip
    DroneShowLEDType_NeoPixel = 7,

    // LED light is to be forwarded to a ProfiLED LED strip
    DroneShowLEDType_ProfiLED = 8,

    // Debug output; LED light RGB codes are sent as MAVLink STATUSTEXT messages
    DroneShowLEDType_Debug = 9
};

class DroneShowLEDFactory
{
public:
    DroneShowLEDFactory();

    /* Do not allow copies */
    DroneShowLEDFactory(const DroneShowLEDFactory &other) = delete;
    DroneShowLEDFactory &operator=(const DroneShowLEDFactory&) = delete;

    /**
     * Creates a new DroneShowLED instance, given the LED type and the number of
     * LEDs on this channel (for NeoPixel or ProfiLED strips).
     */
    DroneShowLED* new_rgb_led_by_type(DroneShowLEDType type, uint8_t num_leds);
};
