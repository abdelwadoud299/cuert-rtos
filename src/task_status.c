/**
 * @file task_status.c
 * @brief FreeRTOS Task 4: STATUS (Priority 2 - Lowest).
 */

#include "tasks.h"
#include "types.h"
#include "config.h"
#include "sync.h"

#include <stdio.h>
#include "esp_timer.h"

TaskHandle_t g_handle_status = NULL;

void Task_Status(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(STATUS_PERIOD_MS);

    UART_Println("[STATUS] Task started. 1 Hz periodic telemetry online.");

    while (1) {
        /* Drift-free periodic delay */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t uptime_sec = now_ms / 1000UL;

        static ActuatorState_t state = {0};
        Sync_GetStateSnapshot(&state);

        uint32_t last_cmd_time_ms = Sync_GetLastCommandTimeMs();
        uint32_t age_ms = (now_ms >= last_cmd_time_ms) ? (now_ms - last_cmd_time_ms) : 0;

        const char *mode_str = (state.mode == SYSTEM_MODE_FAIL_SAFE) ? "FAIL-SAFE [SAFE IDLE]" :
                               (state.mode == SYSTEM_MODE_NORMAL)    ? "NORMAL [ACTIVE]" : "INIT";

        UART_Printf("[STATUS] Uptime: %4lus | LastCmd: %c=%-4d (%4lums ago) | Output: PWM=%3d%% (T:%3d%% S:%-4d B:%3d%%) | Mode: %s\r\n",
                    uptime_sec,
                    state.last_cmd_type,
                    state.last_cmd_value,
                    age_ms,
                    state.effective_pwm_percent,
                    state.throttle_percent,
                    state.steer_value,
                    state.brake_percent,
                    mode_str);
    }
}
