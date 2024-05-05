#include "sdkconfig.h"
#include "main.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_task.h"
#include "esp_system.h"
#include "CanBus.h"

// Pins used for TWAI (CAN bus)
#define CONFIG_TWAI_RX GPIO_NUM_15
#define CONFIG_TWAI_TX GPIO_NUM_16

static const char *tag = "TestCanBus";

void Test()
{
    printf("Test\n");
    CanBus bus(CONFIG_TWAI_RX, CONFIG_TWAI_TX);

    // loop vtaskdelay
    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        twai_status_info_t statusInfo = {};
        bus.GetStatusInfo(&statusInfo);
        ESP_LOGI(tag, "state=%d, msgs_to_tx=%lu, msgs_to_rx=%lu, tx_error_counter=%lu, rx_error_counter=%lu, tx_failed_count=%lu, rx_missed_count=%lu, rx_overrun_count=%lu, arb_lost_count=%lu, bus_error_count=%lu", statusInfo.state, statusInfo.msgs_to_tx, statusInfo.msgs_to_rx, statusInfo.tx_error_counter, statusInfo.rx_error_counter, statusInfo.tx_failed_count, statusInfo.rx_missed_count, statusInfo.rx_overrun_count, statusInfo.arb_lost_count, statusInfo.bus_error_count);

    }
}

extern "C"
{
    void app_main();

}

void app_main()
{
    Test();
};

