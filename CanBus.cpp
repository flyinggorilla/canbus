#include <esp_log.h>
#include <esp_system.h>
#include <esp_task.h>
#include "CanBus.h"

const char tag[] = "CanBus";

// raise a compiler error if  CONFIG_TWAI_ISR_IN_IRAM is not set
#if !CONFIG_TWAI_ISR_IN_IRAM
#error "CONFIG_TWAI_ISR_IN_IRAM must be set in menuconfig"
#endif

#define CAN_BUS_TIMEOUT_MS 10 // at 1Mbit/s the roundtrip of send/response with devices has been observed as approx 300ns

CanBus::CanBus(gpio_num_t twaiRX, gpio_num_t twaiTX)
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(twaiTX, twaiRX, TWAI_MODE_NORMAL);
    generalConfig.alerts_enabled = TWAI_ALERT_ERR_ACTIVE |
                                   TWAI_ALERT_RECOVERY_IN_PROGRESS |
                                   TWAI_ALERT_BUS_RECOVERED |
                                   TWAI_ALERT_ABOVE_ERR_WARN |
                                   TWAI_ALERT_BUS_ERROR |
                                   TWAI_ALERT_TX_FAILED |
                                   TWAI_ALERT_RX_QUEUE_FULL |
                                   TWAI_ALERT_ERR_PASS |
                                   TWAI_ALERT_BUS_OFF |
                                   TWAI_ALERT_RX_FIFO_OVERRUN |
                                   TWAI_ALERT_TX_RETRIED |
                                   TWAI_ALERT_PERIPH_RESET;

    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to install driver");
    }

    // Start TWAI driver
    if (twai_start() != ESP_OK)
    {
        ESP_LOGE(tag, "Failed to start driver");
    }

    twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
}

CanBus::~CanBus()
{
}

// add method to CanBus class to transmit a message
esp_err_t CanBus::TransmitMessage(const twai_message_t *message)
{
    // Transmit a message
    return twai_transmit(message, pdMS_TO_TICKS(CAN_BUS_TIMEOUT_MS));
}

// add method to CanBus class to receive a message
esp_err_t CanBus::ReceiveMessage(twai_message_t *message, unsigned int timeoutMs)
{
    // Receive a message
    esp_err_t ret = twai_receive(message, pdMS_TO_TICKS(timeoutMs));
    if (ret == ESP_OK)
    {
        ESP_LOGD(tag, "Message received, ID is %u, data length code is %d", (unsigned int)message->identifier, message->data_length_code);
        for (int i = 0; i < message->data_length_code; i++)
        {
            ESP_LOGD(tag, "Data byte %d is %d", i, message->data[i]);
        }
    }
    return ret;
}

const char *CanBus::ErrorToMessage(esp_err_t error)
{
    switch (error)
    {
    case ESP_OK:
        return "Transmission successfully queued/initiated";
    case ESP_ERR_INVALID_ARG:
        return "Arguments are invalid";
    case ESP_ERR_TIMEOUT:
        return "Timed out waiting for space on TX queue";
    case ESP_FAIL:
        return "TX queue is disabled and another message is currently transmitting";
    case ESP_ERR_INVALID_STATE:
        return "TWAI driver is not in running state, or is not installed";
    case ESP_ERR_NOT_SUPPORTED:
        return "Listen Only Mode does not support transmissions";
    default:
        return "Unknown error";
    }
}

bool CanBus::ReadStatus()
{
    twai_get_status_info(&statusInfo);
    ESP_LOGI(tag, "state=%d, msgs_to_tx=%lu, msgs_to_rx=%lu, tx_error_counter=%lu, rx_error_counter=%lu, tx_failed_count=%lu, rx_missed_count=%lu, rx_overrun_count=%lu, arb_lost_count=%lu, bus_error_count=%lu", statusInfo.state, statusInfo.msgs_to_tx, statusInfo.msgs_to_rx, statusInfo.tx_error_counter, statusInfo.rx_error_counter, statusInfo.tx_failed_count, statusInfo.rx_missed_count, statusInfo.rx_overrun_count, statusInfo.arb_lost_count, statusInfo.bus_error_count);
    return true;
}

/// @brief Reads CAN Bus alerts and resets them afterwards
/// @return alert flags
uint32_t CanBus::ReadAlerts()
{
    uint32_t alerts = 0;
    if (ESP_OK == twai_read_alerts(&alerts, pdMS_TO_TICKS(0)))
    {
        ESP_LOGD(tag, "alerts=%lu", alerts);
    }
    else
    {
        alerts = 0; // timeout or error
    }
    return alerts;
}
