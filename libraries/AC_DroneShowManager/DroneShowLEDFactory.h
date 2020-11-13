#pragma once

/// @file   AC_DroneShowLEDFactory.h
/// @brief  LED factory class that creates LED instances for the drone show manager module

#include <AP_Notify/RGBLed.h>

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
};

class DroneShowLEDFactory
{
public:
    DroneShowLEDFactory();

    /* Do not allow copies */
    DroneShowLEDFactory(const DroneShowLEDFactory &other) = delete;
    DroneShowLEDFactory &operator=(const DroneShowLEDFactory&) = delete;

    /**
     * Creates a new RGBLed instance, given the LED type.
     */
    RGBLed* new_rgb_led_by_type(DroneShowLEDType type);
};
