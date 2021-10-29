#include "DroneShowLED_I2C.h"

extern const AP_HAL::HAL& hal;

DroneShowLED_I2C::DroneShowLED_I2C(uint8_t bus, uint8_t addr)
    : DroneShowLED(),
    _bus(bus), _addr(addr), _send_required(false),
    _last_red(0), _last_green(0), _last_blue(0)
{
}

bool DroneShowLED_I2C::init()
{
    _dev = std::move(hal.i2c_mgr->get_device(_bus, _addr));
    if (!_dev) {
        return false;
    }

    // Register a periodic callback to be called at 50 Hz; this will be used to
    // initiate I2C transfers if needed
    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&DroneShowLED_I2C::_timer, void));
    return true;
}

void DroneShowLED_I2C::set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    WITH_SEMAPHORE(_sem);

    if (_last_red == red && _last_green == green && _last_blue == blue) {
        return;
    }

    _last_red = red;
    _last_green = green;
    _last_blue = blue;
    _send_required = true;
}

void DroneShowLED_I2C::_timer(void)
{
    uint8_t msg[3];

    WITH_SEMAPHORE(_sem);

    if (!_send_required) {
        return;
    }

    hal.console->printf("Foo\n");

    _send_required = false;
    msg[0] = _last_red;
    msg[1] = _last_green;
    msg[2] = _last_blue;
    _dev->transfer(msg, sizeof(msg), nullptr, 0);
}
