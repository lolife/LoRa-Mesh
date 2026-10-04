#include "mk_espmesh_lib.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace {
struct Packet {
    uint8_t destination[ESP_NOW_ETH_ALEN];
    uint8_t data[sizeof(StatusMessage)];
    size_t length;
};
QueueHandle_t txQueue = nullptr;
TaskHandle_t txTask = nullptr;
constexpr size_t QUEUE_LENGTH = 32;
constexpr size_t PRIORITY_RESERVE = 4;

void transmit(void*) {
    Packet packet;
    unsigned long lastErrorLog = 0;
    for (;;) {
        if (xQueueReceive(txQueue, &packet, portMAX_DELAY) != pdTRUE) continue;
        // Only this worker calls esp_now_send. The completion callback releases
        // it before another packet is submitted, including relay fan-out.
        ulTaskNotifyTake(pdTRUE, 0);
        esp_err_t result = ESP_ERR_ESPNOW_NO_MEM;
        for (int attempt = 0; attempt < 3 && result == ESP_ERR_ESPNOW_NO_MEM; ++attempt) {
            result = esp_now_send(packet.destination, packet.data, packet.length);
            if (result == ESP_ERR_ESPNOW_NO_MEM) vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (result == ESP_OK) {
            if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
                ESP_LOGW(TAG, "%s", "Mesh TX completion timed out");
            }
        } else if (millis() - lastErrorLog >= 10000) {
            lastErrorLog = millis();
            ESP_LOGW(TAG, "Mesh TX failed to %s: %s (%d)",
                     stringMacAddress(packet.destination).c_str(), esp_err_to_name(result), result);
        }
    }
}
}

bool initMeshTx() {
    if (txQueue) return true;
    txQueue = xQueueCreate(QUEUE_LENGTH, sizeof(Packet));
    if (!txQueue) return false;
    if (xTaskCreate(transmit, "mesh-tx", 3072, nullptr, 1, &txTask) != pdPASS) {
        vQueueDelete(txQueue);
        txQueue = nullptr;
        return false;
    }
    return true;
}

void meshTxComplete() {
    if (txTask) xTaskNotifyGive(txTask);
}

bool queueMeshPacket(const uint8_t* destination, const uint8_t* data, size_t length, bool priority) {
    if (!txQueue || !destination || !data || length > sizeof(Packet::data)) return false;
    // Leave room for priority packets even
    // during a burst of ordinary mesh relays. All producers are nonblocking.
    if (!priority && uxQueueSpacesAvailable(txQueue) <= PRIORITY_RESERVE) return false;
    Packet packet = {};
    memcpy(packet.destination, destination, sizeof(packet.destination));
    memcpy(packet.data, data, length);
    packet.length = length;
    return (priority ? xQueueSendToFront(txQueue, &packet, 0)
                     : xQueueSend(txQueue, &packet, 0)) == pdTRUE;
}
