/**
 * @file main.c
 * @brief Application entry point and FreeRTOS task launcher.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "config.h"
#include "types.h"
#include "sync.h"
#include "actuator.h"
#include "tasks.h"

static const char *TAG = "MAIN";

static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate           = SERIAL_BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
}

static void print_banner(void)
{
    UART_Println("\r\n========================================================");
    UART_Println("  REAL-TIME ACTUATION & FAIL-SAFE SUPERVISOR (ESP32)   ");
    UART_Println("      FreeRTOS Autonomous Automotive Gateway Node       ");
    UART_Println("========================================================");
    UART_Println("  System online. 4 FreeRTOS tasks initialized:          ");
    UART_Println("   [P5] COMMAND_RX : UART command ingestion             ");
    UART_Println("   [P4] ACTUATE    : Drives PWM, enforces BRAKE safety  ");
    UART_Println("   [P3] WATCHDOG   : 500ms timeout -> Safe-Idle blink   ");
    UART_Println("   [P2] STATUS     : 1 Hz telemetry background logger   ");
    UART_Println("========================================================\r\n");
}

void app_main(void)
{
    init_uart();
    Actuator_Init();

    if (Sync_Init() != pdPASS) {
        ESP_LOGE(TAG, "Failed to initialize FreeRTOS sync primitives");
        return;
    }

    print_banner();

    /* Pin real-time tasks to Core 1 to avoid Core 0 wireless/flash interrupt jitter */
    xTaskCreatePinnedToCore(Task_CommandRx, "COMMAND_RX", STACK_SIZE_COMMAND_RX, NULL, TASK_PRIORITY_COMMAND_RX, &g_handle_command_rx, 1);
    xTaskCreatePinnedToCore(Task_Actuate,   "ACTUATE",    STACK_SIZE_ACTUATE,    NULL, TASK_PRIORITY_ACTUATE,    &g_handle_actuate,    1);
    xTaskCreatePinnedToCore(Task_Watchdog,  "WATCHDOG",   STACK_SIZE_WATCHDOG,   NULL, TASK_PRIORITY_WATCHDOG,  &g_handle_watchdog,   1);
    xTaskCreatePinnedToCore(Task_Status,    "STATUS",     STACK_SIZE_STATUS,     NULL, TASK_PRIORITY_STATUS,    &g_handle_status,     1);
}
