#include "DroneShowLED_UART.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

DroneShowLED_UART::DroneShowLED_UART(uint8_t uart_index) : DroneShowLED(),
    _uart_index(uart_index) // the UART port we need to use
{
}

bool DroneShowLED_UART::init()
{
    return true;
}

bool DroneShowLED_UART::set_raw_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	AP_HAL::UARTDriver *uart = hal.serial(_uart_index);
    if (uart != nullptr) {
        send_rgb_command_to_uart(uart, red, green, blue);
    }
    return true;
}
