/**
 * @file task_actuate.c
 * @brief FreeRTOS Task 2: ACTUATE (Priority 4 - Medium).
 */

#include "tasks.h"
#include "types.h"
#include "config.h"
#include "sync.h"
#include "actuator.h"

#include <stdio.h>
#include <stdbool.h>

TaskHandle_t g_handle_actuate = NULL;

void Task_Actuate(void *pvParameters)
{
    (void)pvParameters;

    Command_t rx_cmd;
    int16_t current_throttle = 0;
    int16_t current_steer    = 0;
    int16_t current_brake    = 0;
    uint8_t effective_output = 0;
    bool    brake_override   = false;

    UART_Println("[ACTUATE] Task started. Waiting for commands from queue...");

    while (1) {
        if (xQueueReceive(g_cmd_queue, &rx_cmd, portMAX_DELAY) == pdTRUE) {
            switch (rx_cmd.type) {
                case CMD_TYPE_THROTTLE: current_throttle = rx_cmd.value; break;
                case CMD_TYPE_STEER:    current_steer = rx_cmd.value;    break;
                case CMD_TYPE_BRAKE:    current_brake = rx_cmd.value;    break;
                case CMD_TYPE_PING:                                      break;
                default:                                                 break;
            }

            /*
             * Automotive Safety Interlock:
             * On any BRAKE > 0, output is clamped to 0 immediately.
             */
            if (current_brake > 0) {
                brake_override   = true;
                effective_output = 0;
            } else {
                brake_override   = false;
                effective_output = (uint8_t)current_throttle;
            }

            ActuatorState_t state_snapshot = {0};
            if (Sync_GetStateSnapshot(&state_snapshot)) {
                if (state_snapshot.mode == SYSTEM_MODE_FAIL_SAFE) {
                    Sync_SetSystemMode(SYSTEM_MODE_NORMAL);
                }
            }

            Actuator_SetOutput(effective_output);

            Sync_UpdateActuatorState(current_throttle, current_steer, current_brake,
                                     effective_output, brake_override,
                                     rx_cmd.type, rx_cmd.value, rx_cmd.timestamp_ms);

            UART_Printf("[ACTUATE] Executed: Cmd='%c' Val=%4d | State: Throttle=%3d%% Steer=%4d Brake=%3d%% | PWM=%3d%% %s\r\n",
                        rx_cmd.type,
                        rx_cmd.value,
                        current_throttle,
                        current_steer,
                        current_brake,
                        effective_output,
                        brake_override ? "[BRAKE OVERRIDE ACTIVE]" : "");
        }
    }
}
