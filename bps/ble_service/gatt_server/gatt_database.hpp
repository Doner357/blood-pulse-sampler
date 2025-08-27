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

            struct Command {
                static constexpr std::uint16_t kValue           = ATT_CHARACTERISTIC_652C47C1_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kUserDescription = ATT_CHARACTERISTIC_652C47C1_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };

            struct MachineStatus {
                static constexpr std::uint16_t kValue               = ATT_CHARACTERISTIC_652C47C2_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kClientConfiguration = ATT_CHARACTERISTIC_652C47C2_C653_41BC_8828_30200EF3350A_01_CLIENT_CONFIGURATION_HANDLE;
                static constexpr std::uint16_t kUserDescription     = ATT_CHARACTERISTIC_652C47C2_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
            };

            struct PulseValue {
                static constexpr std::uint16_t kValue               = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_VALUE_HANDLE;
                static constexpr std::uint16_t kClientConfiguration = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_CLIENT_CONFIGURATION_HANDLE;
                static constexpr std::uint16_t kUserDescription     = ATT_CHARACTERISTIC_652C47C3_C653_41BC_8828_30200EF3350A_01_USER_DESCRIPTION_HANDLE;
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
            0x18, 0x00, 0x02, 0x00, 0x06, 0x00, 0x2a, 0x2b, 0xef, 0x29, 0x8b, 0x5d, 0x77, 0x1c, 0x19, 0x28, 0x7c, 0xd8, 0xf0, 0xf5, 0x8a, 0xaf, 0x04, 0xe1, 
            // First custom service: Pulse Sampler
            // 0x0007 PRIMARY_SERVICE-652C47C0-C653-41BC-8828-30200EF3350A
            0x18, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x28, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc0, 0x47, 0x2c, 0x65, 
            // --- Cuntom Service's Characteristics ---
            // Characteristic A: Command Packet
            // write, dynamic
            // 0x0008 CHARACTERISTIC-652C47C1-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            0x1b, 0x00, 0x02, 0x00, 0x08, 0x00, 0x03, 0x28, 0x08, 0x09, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc1, 0x47, 0x2c, 0x65, 
            // 0x0009 VALUE CHARACTERISTIC-652C47C1-C653-41BC-8828-30200EF3350A - DYNAMIC | WRITE
            // WRITE_ANYBODY
            0x16, 0x00, 0x08, 0x03, 0x09, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc1, 0x47, 0x2c, 0x65, 
            // 0x000a USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x0a, 0x00, 0x01, 0x29, 
            // Characteristic B: Machine Status Packet
            // read, dynamic, with notifications
            // 0x000b CHARACTERISTIC-652C47C2-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            0x1b, 0x00, 0x02, 0x00, 0x0b, 0x00, 0x03, 0x28, 0x12, 0x0c, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc2, 0x47, 0x2c, 0x65, 
            // 0x000c VALUE CHARACTERISTIC-652C47C2-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            // READ_ANYBODY
            0x16, 0x00, 0x02, 0x03, 0x0c, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc2, 0x47, 0x2c, 0x65, 
            // 0x000d CLIENT_CHARACTERISTIC_CONFIGURATION
            // READ_ANYBODY, WRITE_ANYBODY
            0x0a, 0x00, 0x0e, 0x01, 0x0d, 0x00, 0x02, 0x29, 0x00, 0x00, 
            // 0x000e USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x0e, 0x00, 0x01, 0x29, 
            // Characteristic D: Pulse Data Packet
            // read only, dynamic, with notifications
            // 0x000f CHARACTERISTIC-652C47C3-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            0x1b, 0x00, 0x02, 0x00, 0x0f, 0x00, 0x03, 0x28, 0x12, 0x10, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc3, 0x47, 0x2c, 0x65, 
            // 0x0010 VALUE CHARACTERISTIC-652C47C3-C653-41BC-8828-30200EF3350A - DYNAMIC | READ | NOTIFY
            // READ_ANYBODY
            0x16, 0x00, 0x02, 0x03, 0x10, 0x00, 0x0a, 0x35, 0xf3, 0x0e, 0x20, 0x30, 0x28, 0x88, 0xbc, 0x41, 0x53, 0xc6, 0xc3, 0x47, 0x2c, 0x65, 
            // 0x0011 CLIENT_CHARACTERISTIC_CONFIGURATION
            // READ_ANYBODY, WRITE_ANYBODY
            0x0a, 0x00, 0x0e, 0x01, 0x11, 0x00, 0x02, 0x29, 0x00, 0x00, 
            // 0x0012 USER_DESCRIPTION-READ
            // READ_ANYBODY, WRITE_ANYBODY
            0x08, 0x00, 0x0a, 0x01, 0x12, 0x00, 0x01, 0x29, 
            // END
            0x00, 0x00
        );
    };
};


} // namespace bps::ble

#endif // C_GATT_DATABASE_HPP