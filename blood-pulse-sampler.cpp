#include <pico/stdlib.h>

#include "bps/ble_service/ble_service.hpp"
#include "bps/sampler_service/sampler_service.hpp"
#include "bps/logger/logger.hpp"

int main() {
    bps::logger::initializeLogger();
    sleep_ms(1000);
    
    BPS_LOG("Start BPS!\n");

    auto& ble_service = bps::ble::BleService::getInstance();
    ble_service.initialize();

    auto& sampler_service = bps::sampler::SamplerService::getInstance();
    sampler_service.initialize();

    sampler_service.registerPulseValueQueue(ble_service.getPulseValueQueueRef());
    ble_service.registerCommandQueue(sampler_service.getCommandQueueRef());

    ble_service.createTask(2);
    sampler_service.createTask(1);

    vTaskStartScheduler();

    return 0;
}