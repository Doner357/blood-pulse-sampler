#include <pico/stdlib.h>

#include "bps/ble_service/ble_service.hpp"
#include "bps/pneumatic/pneumatic.hpp"

int main() {

    auto& ble_service = bps::ble::BleService::getInstance();
    ble_service.initialize();

    auto& pneumatic_service = bps::pneumatic::PneumaticService::getInstance();
    pneumatic_service.initialize();

    pneumatic_service.registerPulseValueQueue(ble_service.getPulseValueQueue());
    ble_service.registerCommandQueue(pneumatic_service.getCommandQueue());

    ble_service.createTask(2);
    pneumatic_service.createTask(1);

    vTaskStartScheduler();

    return 0;
}