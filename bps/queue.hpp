#ifndef BPS_QUEUE_HPP
#define BPS_QUEUE_HPP

#include <FreeRTOS.h>
#include <queue.h>

namespace bps {

template<typename T, UBaseType_t Length>
class StaticQueue {
    public:
        using ContentType = T;
        // Constructor
        StaticQueue() noexcept {
            this->queue_handle = xQueueCreateStatic(
                Length,
                sizeof(T),
                reinterpret_cast<uint8_t*>(buffer.data()),
                &this->static_queue_cb
            );
            configASSERT(this->queue_handle != nullptr);
        }

        // Destructor to release the queue handle
        ~StaticQueue() {
            if (this->queue_handle != nullptr) {
                vQueueDelete(this->queue_handle);
            }
        }

        StaticQueue(StaticQueue const&) = delete;
        StaticQueue& operator=(StaticQueue const&) = delete;
        StaticQueue(StaticQueue&& other) = delete;
        StaticQueue& operator=(StaticQueue&& other) = delete;

        // --- API Methods ---

        // Send by copying a const reference
        bool send(T const& object, TickType_t wait_ms) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueSend(
                    this->queue_handle, 
                    reinterpret_cast<void const*>(&object), 
                    wait_ms
                ) != pdPASS
            ) return false;
            return true;
        }

        bool sendFromIsr(T const& object, BaseType_t* higher_priority_task_to_woken) noexcept {
            if (
                this->queue_handle == nullptr ||
                xQueueSendFromISR(
                    this->queue_handle,
                    reinterpret_cast<void const*>(&object),
                    higher_priority_task_to_woken
                ) != pdPASS
            ) return false;
            return true;
        }

        // Receive an object
        bool receive(T& receive_buffer, TickType_t wait_ms) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueReceive(
                    this->queue_handle,
                    reinterpret_cast<void*>(&receive_buffer),
                    pdTICKS_TO_MS(wait_ms)
                ) != pdPASS
            ) return false;
            return true;
        }

        // Get the raw FreeRTOS queue handle
        QueueHandle_t getFreeRTOSQueueHandle() const noexcept {
            return this->queue_handle;
        }

        // Helper to check if the queue was created successfully
        bool isValid() const noexcept {
            return this->queue_handle != nullptr;
        }

        // Get the length of the queue
        static constexpr UBaseType_t length() {
            return Length;
        }

        UBaseType_t size() const noexcept {
            return uxQueueMessagesWaiting(this->queue_handle);
        }

    private:
        std::array<std::byte, Length * sizeof(T)> buffer{};
        StaticQueue_t static_queue_cb{};
        QueueHandle_t queue_handle{nullptr};
};

} // namespace bps

#endif // BPS_QUEUE_HPP