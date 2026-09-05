/**
 * @file task_watchdog.c
 * @brief FreeRTOS Task 3: WATCHDOG / FAIL-SAFE (Priority 3 - Low).
 */

#include "tasks.h"
#include "types.h"
#include "config.h"
#include "sync.h"
#include "actuator.h"

#include <stdio.h>
#include <stdbool.h>
#include "esp_timer.h"

TaskHandle_t g_handle_watchdog = NULL;

void Task_Watchdog(void *pvParameters)
{
    (void)pvParameters;

    bool failsafe_active = false;
    bool blink_toggle_state = false;
    uint32_t last_blink_time_ms = 0;

    UART_Println("[WATCHDOG] Task started. Supervising link health (500ms timeout)...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t last_cmd_ms = Sync_GetLastCommandTimeMs();
        uint32_t elapsed_ms = (now_ms >= last_cmd_ms) ? (now_ms - last_cmd_ms) : 0;

        if (elapsed_ms >= WATCHDOG_TIMEOUT_MS) {
            if (!failsafe_active) {
                failsafe_active = true;
                Sync_SetSystemMode(SYSTEM_MODE_FAIL_SAFE);

                /* Safety invariant: log exact link loss alert */
                UART_Println("LINK LOST , failing safe");

                Actuator_SetDigitalLevel(false);
                blink_toggle_state = false;
                last_blink_time_ms = now_ms;
            }

            /* Safe-idle blink pattern: 2 Hz (250ms ON / 250ms OFF) */
            if ((now_ms - last_blink_time_ms) >= SAFE_IDLE_BLINK_HALF_PERIOD_MS) {
                blink_toggle_state = !blink_toggle_state;
                Actuator_SetDigitalLevel(blink_toggle_state);
                last_blink_time_ms = now_ms;
            }
        } else {
            /* Automatic recovery upon new valid command */
            if (failsafe_active) {
                failsafe_active = false;
                Sync_SetSystemMode(SYSTEM_MODE_NORMAL);

                UART_Println("[WATCHDOG] LINK RESTORED: Resuming normal actuation");

                ActuatorState_t current_state = {0};
                if (Sync_GetStateSnapshot(&current_state)) {
                    Actuator_SetOutput(current_state.effective_pwm_percent);
                }
            }
        }
    }
}
