#include "AP_PowerManagement_UART.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_PowerManagement_UART::AP_PowerManagement_UART(
    AP_PowerManagement& frontend, uint8_t uart_index
) : AP_PowerManagement_Backend(frontend), _uart_index(uart_index)
{
}

AP_HAL::UARTDriver* AP_PowerManagement_UART::get_uart() const {
    return hal.serial(_uart_index);
}
