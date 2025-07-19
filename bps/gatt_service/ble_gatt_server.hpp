#ifndef BLE_GATT_SERVER_HPP
#define BLE_GATT_SERVER_HPP

#include <btstack.h>

#include <cstdint>
#include <cstddef>
#include <array>

#include "gatt_database.hpp"

namespace bps::gatt {

// TODO: Implement a interactable GATT Server object
class GattServer {
    public:
        
    private:
        // Connection handle for service
        hci_con_handle_t      con_handle;
};
    
} // namespace bps

#endif // BLE_GATT_SERVER_HPP