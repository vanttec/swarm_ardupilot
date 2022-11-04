#include "DroneShowLED_I2C.h"

extern const AP_HAL::HAL& hal;

DroneShowLED_I2C::DroneShowLED_I2C(uint8_t bus, uint8_t addr)
    : DroneShowLED(),
    _bus(bus), _addr(addr), _send_required(false)
{
}

bool DroneShowLED_I2C::init()
{
    _dev = hal.i2c_mgr->get_device(_bus, _addr);
    if (!_dev) {
        return false;
    }

    // Register a periodic callback to be called at 50 Hz; this will be used to
    // initiate I2C transfers if needed
    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&DroneShowLED_I2C::_timer, void));
    return true;
}

bool DroneShowLED_I2C::set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    WITH_SEMAPHORE(_sem);

    _msg[0] = red;
    _msg[1] = green;
    _msg[2] = blue;
    _send_required = true;

    return true;
}

void DroneShowLED_I2C::_timer(void)
{
    uint8_t msg[3];

    WITH_SEMAPHORE(_sem);

    if (!_send_required) {
        return;
    }

    _send_required = false;
    memcpy(msg, _msg, sizeof(msg));

    _dev->transfer(msg, sizeof(msg), nullptr, 0);
}
