#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H

// -- Core Feature Flags --
#ifndef ENABLE_BLE
#error Please link to pico_btstack_ble
#endif

// Enable LE Peripheral role
#define ENABLE_LE_PERIPHERAL

// Enable logging for debugging
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Project is Peripheral Only, Central role is not needed
#define MAX_NR_GATT_CLIENTS 0

// -- Buffers and Connection Management --

// Required for CYW43 driver compatibility
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// [Req 1] Allow only one connection at a time
#define MAX_NR_HCI_CONNECTIONS 1

// Enable Data Length Extension (DLE) to maximize throughput
#define HCI_ACL_PAYLOAD_SIZE (255 + 4)

// [Req 2] Increase Host and Controller buffers to support high-frequency transport
// Default is 3. Increased to 8 to significantly improve throughput by creating a data pipeline.
#define MAX_NR_CONTROLLER_ACL_BUFFERS 8
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// Enable and configure HCI Controller-to-Host Flow Control for Pico W to prevent bus overruns
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN (255+4)
#define HCI_HOST_ACL_PACKET_NUM 8
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// -- Security and Database Settings --
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 16
#define MAX_NR_LE_DEVICE_DB_ENTRIES 16

// NVM (Non-Volatile Memory) configuration for storing pairing keys
#define NVM_NUM_DEVICE_DB_ENTRIES 16
#define NVM_NUM_LINK_KEYS 16

// ATT Database size. 512 bytes is more than enough for the custom service.
#define MAX_ATT_DB_SIZE 512

// -- BTstack HAL (Hardware Abstraction Layer) Configuration --
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS 1000
#define ENABLE_SOFTWARE_AES128
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS

#endif