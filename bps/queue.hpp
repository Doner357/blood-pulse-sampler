#ifndef BPS_QUEUE_HPP
#define BPS_QUEUE_HPP

#include <FreeRTOS.h>
#include <queue.h>

#include <cstddef>
#include <cstdint>
#include <array>
#include <algorithm>
#include <concepts>
#include <expected>

namespace bps {

template<typename Q>
concept QueueType = requires(
    Q queue, 
    typename Q::ContentType content,
    TickType_t wait_tick
) {
    { queue.send(content, pdTICKS_TO_MS(wait_tick)) } noexcept -> std::same_as<bool>;
    { queue.receive(content, pdTICKS_TO_MS(wait_tick)) } noexcept -> std::same_as<bool>;
    { queue.getFreeRTOSQueueHandle() } noexcept -> std::same_as<QueueHandle_t>;
};

template<typename Q>
concept StaticQueueType = QueueType<Q> && requires() {
    { Q::length() } -> std::same_as<UBaseType_t>;
};

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
        bool send(T const& object, TickType_t wait_tick) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueSend(
                    this->queue_handle, 
                    reinterpret_cast<void const*>(&object), 
                    wait_tick
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
        bool receive(T& receive_buffer, TickType_t wait_tick) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueReceive(
                    this->queue_handle,
                    reinterpret_cast<void*>(&receive_buffer),
                    wait_tick
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

template <typename T>
class QueueReference {
    public:
        QueueReference() = default;

        template<QueueType Q>
        QueueReference(Q const& queue) {
            this->queue_handle = queue.getFreeRTOSQueueHandle();
        }

        // --- API Methods ---

        // Send by copying a const reference
        bool send(T const& object, TickType_t wait_tick) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueSend(
                    this->queue_handle, 
                    reinterpret_cast<void const*>(&object), 
                    wait_tick
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
        bool receive(T& receive_buffer, TickType_t wait_tick) noexcept {
            if (
                this->queue_handle == nullptr || 
                xQueueReceive(
                    this->queue_handle,
                    reinterpret_cast<void*>(&receive_buffer),
                    wait_tick
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

        UBaseType_t size() const noexcept {
            return uxQueueMessagesWaiting(this->queue_handle);
        }

    private:
        QueueHandle_t queue_handle{ nullptr };
};
template <QueueType Q>
QueueReference(Q const&) -> QueueReference<typename Q::ContentType>;

template<StaticQueueType... Qs>
class StaticQueueSet {
        // The combined length of the all queues and that will be
        // added to the queue set.
        static constexpr std::size_t kCombinedLength = (Qs::length() + ...);
        // Define the maximum item size for the queue set
        static constexpr std::size_t kMaxItemSize = std::max({ (sizeof(typename Qs::ContentType), ...) });
        // Check the total size of static storage can't acceed the size of UBaseType_t
        static_assert((kCombinedLength * kMaxItemSize) > sizeof(UBaseType_t));
    public:
        StaticQueueSet(Qs&...  queues) {
            this->queue_set_handle = xQueueCreateSetStatic(
                                    kCombinedLength,
                                    reinterpret_cast<uint8_t*>(this->buffer.data()),
                                    &this->static_queue_cb
                                );
            configASSERT(this->queue_set_handle);

            (xQueueAddToSet(queues.getFreeRTOSQueueHandle(), this->queue_set_handle), ...);
        }

        std::expected<QueueHandle_t, std::nullptr_t> selectFromSet(TickType_t wait_tick) const noexcept {
            QueueSetMemberHandle_t handle = xQueueSelectFromSet(
                                                this->queue_set_handle,
                                                wait_tick
                                            );
            if (!handle) {
                return std::unexpected(nullptr);
            }
            return reinterpret_cast<QueueHandle_t>(handle);
        }

        StaticQueueSet(StaticQueueSet const&) = delete;
        StaticQueueSet& operator=(StaticQueueSet const&) = delete;
        StaticQueueSet(StaticQueueSet&& other) = delete;
        StaticQueueSet& operator=(StaticQueueSet&& other) = delete;
    private:
        // Array for static storage
        std::array<std::byte, kCombinedLength * kMaxItemSize> buffer{};
        StaticQueue_t static_queue_cb{};
        QueueSetHandle_t queue_set_handle{nullptr};
};

template<typename... Qs>
StaticQueueSet<Qs...> makeQueueSet(Qs&... queues) noexcept {
    return StaticQueueSet<Qs...>(queues...);
}

} // namespace bps

#endif // BPS_QUEUE_HPP