#ifndef BPS_GATT_DATABASE_HPP
#define BPS_GATT_DATABASE_HPP

#include <cstdint>

#include "gatt_database.h"
#include "utils.hpp"

namespace bps::ble::gatt {

struct Att {
    // Wrap ATT C macros into constant struct type
    struct Handle {
        struct Gap {
            static constexpr std::uint16_t kStart      = ATT_SERVICE_GAP_SERVICE_START_HANDLE;
            static constexpr std::uint16_t kEnd        = ATT_SERVICE_GAP_SERVICE_END_HANDLE;
            static constexpr std::uint16_t kDeviceName = ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE;
        };

        struct Gatt {
            static constexpr std::uint16_t kStart        = ATT_SERVICE_GATT_SERVICE_START_HANDLE;
            static constexpr std::uint16_t kEnd          = ATT_SERVICE_GATT_SERVICE_END_HANDLE;
            static constexpr std::uint16_t kDatabaseHash = ATT_CHARACTERISTIC_GATT_DATABASE_HASH_01_VALUE_HANDLE;
        };

        struct CustomCharacteristic {
            static constexpr std::uint16_t kStart = ATT_SERVICE_652C47C0_C653_41BC_8828_30200EF3350A_START_HANDLE;
            static constexpr std::uint16_t kEnd   = ATT_SERVICE_652C47C0_C653_41BC_8828_30200EF3350A_END_HANDLE;

            struct Action {
                static constexpr std::uint16_t kValue           = ATT_CHARACTERISTIC_652C47C1_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kUserDescription = ATT_CHARACTERISTIC_652C47C1_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };

            struct PressureBaseValue {
                static constexpr std::uint16_t kValue           = ATT_CHARACTERISTIC_652C47C2_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kUserDescription = ATT_CHARACTERISTIC_652C47C2_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };

            struct MachineStatus {
                static constexpr std::uint16_t kValue               = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kClientConfiguration = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_CLIENT_CONFIGURATION_HANDLE;
                static constexpr std::uint16_t kUserDescription     = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };

            struct PulseDataSet {
                static constexpr std::uint16_t kValue               = ATT_CHARACTERISTIC_652C47C4_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kClientConfiguration = ATT_CHARACTERISTIC_652C47C4_C653_41BC_8828_30200EF3350A_01_CLIENT_CONFIGURATION_HANDLE;
                static constexpr std::uint16_t kUserDescription     = ATT_CHARACTERISTIC_652C47C4_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };
        };
    };

    // Wrap database into C++ style constexpr array
    struct Database {
        static constexpr auto kProfileData = makeBytes(
            // ATT DB Version
            1,

            // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
            0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18, 
            // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
            0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a, 
            // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ -'PULSE_SAMPLER_SERVER'
            // READ_ANYBODY
            0x1c, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x50, 0x55, 0x4c, 0x53, 0x45, 0x5f, 0x53, 0x41, 0x4d, 0x50, 0x4c, 0x45, 0x52, 0x5f, 0x53, 0x45, 0x52, 0x56, 0x45, 0x52, 
            // 0x0004 PRIMARY_SERVICE-GATT_SERVICE
            0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x01, 0x18, 
            // 0x0005 CHARACTERISTIC-GATT_DATABASE_HASH - READ
            0x0d, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x02, 0x06, 0x00, 0x2a, 0x2b, 
            // 0x0006 VALUE CHARACTERISTIC-GATT_DATABASE_HASH - READ -''
            // READ_ANYBODY
            0x18, 0x00, 0x02, 0x00, 0x06, 0x00, 0x2a, 0x2b, 0xd2, 0x16, 0x06, 0xc2, 0x31, 0x00, 0x2c, 0x6c, 0xe9, 0x0e, 0x4b, 0x7e, 0xe1, 0x10, 0x7f, 0xf0, 
            // First custom service: Pulse Sampler
            // 0x0007 PRIMARY_SERVICE-652C47C0-C653-41BC-8828-30200EF3350A
            0x18, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x28, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc0, 0x47, 0x2c, 0x65, 
            // --- Cuntom Service's Characteristics ---
            // Characteristic A: Action
            // write, dynamic
            // 0x0008 CHARACTERISTIC-652C47C1-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            0x1b, 0x00, 0x02, 0x00, 0x08, 0x00, 0x03, 0x28, 0x08, 0x09, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc1, 0x47, 0x2c, 0x65, 
            // 0x0009 VALUE CHARACTERISTIC-652C47C1-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            // WRITE_ANYBODY
            0x16, 0x00, 0x08, 0x03, 0x09, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc1, 0x47, 0x2c, 0x65, 
            // 0x000a USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x0a, 0x00, 0x01, 0x29, 
            // Characteristic B: Pressure base value of [float, middle, deep]
            // write, dynamic
            // 0x000b CHARACTERISTIC-652C47C2-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            0x1b, 0x00, 0x02, 0x00, 0x0b, 0x00, 0x03, 0x28, 0x08, 0x0c, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc2, 0x47, 0x2c, 0x65, 
            // 0x000c VALUE CHARACTERISTIC-652C47C2-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            // WRITE_ANYBODY
            0x16, 0x00, 0x08, 0x03, 0x0c, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc2, 0x47, 0x2c, 0x65, 
            // 0x000d USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x0d, 0x00, 0x01, 0x29, 
            // Characteristic C: Machine Status
            // read, dynamic, with notifications
            // 0x000e CHARACTERISTIC-652C47C3-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            0x1b, 0x00, 0x02, 0x00, 0x0e, 0x00, 0x03, 0x28, 0x12, 0x0f, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc3, 0x47, 0x2c, 0x65, 
            // 0x000f VALUE CHARACTERISTIC-652C47C3-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            // READ_ANYBODY
            0x16, 0x00, 0x02, 0x03, 0x0f, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc3, 0x47, 0x2c, 0x65, 
            // 0x0010 CLIENT_CHARACTERISTIC_CONFIGURATION
            // READ_ANYBODY, WRITE_ANYBODY
            0x0a, 0x00, 0x0e, 0x01, 0x10, 0x00, 0x02, 0x29, 0x00, 0x00, 
            // 0x0011 USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x11, 0x00, 0x01, 0x29, 
            // Characteristic D: Pulse data set
            // read only, dynamic, with notifications
            // 0x0012 CHARACTERISTIC-652C47C4-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            0x1b, 0x00, 0x02, 0x00, 0x12, 0x00, 0x03, 0x28, 0x12, 0x13, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc4, 0x47, 0x2c, 0x65, 
            // 0x0013 VALUE CHARACTERISTIC-652C47C4-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            // READ_ANYBODY
            0x16, 0x00, 0x02, 0x03, 0x13, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc4, 0x47, 0x2c, 0x65, 
            // 0x0014 CLIENT_CHARACTERISTIC_CONFIGURATION
            // READ_ANYBODY, WRITE_ANYBODY
            0x0a, 0x00, 0x0e, 0x01, 0x14, 0x00, 0x02, 0x29, 0x00, 0x00, 
            // 0x0015 USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x15, 0x00, 0x01, 0x29, 
            // END
            0x00, 0x00
        );
    };
};


} // namespace bps::ble

#endif // C_GATT_DATABASE_HPP