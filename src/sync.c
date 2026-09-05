/**
 * @file sync.c
 * @brief Thread synchronization, shared state, and safe queue ingestion.
 */

#include "sync.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_timer.h"

QueueHandle_t     g_cmd_queue   = NULL;
SemaphoreHandle_t g_state_mutex = NULL;
SemaphoreHandle_t g_uart_mutex  = NULL;

static volatile uint32_t s_last_valid_cmd_time_ms = 0;

static ActuatorState_t s_actuator_state = {
    .throttle_percent        = 0,
    .steer_value             = 0,
    .brake_percent           = 0,
    .effective_pwm_percent   = 0,
    .brake_override_active   = false,
    .mode                    = SYSTEM_MODE_INIT,
    .last_cmd_type           = '-',
    .last_cmd_value          = 0,
    .last_cmd_timestamp_ms   = 0,
    .total_commands_received = 0,
    .total_commands_dropped  = 0
};

BaseType_t Sync_Init(void)
{
    g_cmd_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Command_t));
    if (g_cmd_queue == NULL) return pdFAIL;

    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) return pdFAIL;

    g_uart_mutex = xSemaphoreCreateMutex();
    if (g_uart_mutex == NULL) return pdFAIL;

    s_last_valid_cmd_time_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return pdPASS;
}

void Sync_UpdateLastCommandTimeMs(uint32_t now_ms)
{
    s_last_valid_cmd_time_ms = now_ms;
}

uint32_t Sync_GetLastCommandTimeMs(void)
{
    return s_last_valid_cmd_time_ms;
}

bool Sync_GetStateSnapshot(ActuatorState_t *out_state)
{
    if (out_state == NULL) return false;

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(out_state, (const void*)&s_actuator_state, sizeof(ActuatorState_t));
        xSemaphoreGive(g_state_mutex);
        return true;
    }
    return false;
}

void Sync_UpdateActuatorState(int16_t throttle, int16_t steer, int16_t brake,
                             uint8_t effective_pwm, bool brake_override,
                             char last_cmd_type, int16_t last_cmd_val,
                             uint32_t last_cmd_timestamp)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_actuator_state.throttle_percent      = throttle;
        s_actuator_state.steer_value           = steer;
        s_actuator_state.brake_percent         = brake;
        s_actuator_state.effective_pwm_percent = effective_pwm;
        s_actuator_state.brake_override_active = brake_override;
        s_actuator_state.last_cmd_type         = last_cmd_type;
        s_actuator_state.last_cmd_value        = last_cmd_val;
        s_actuator_state.last_cmd_timestamp_ms = last_cmd_timestamp;
        s_actuator_state.total_commands_received++;
        xSemaphoreGive(g_state_mutex);
    }
}

void Sync_SetSystemMode(SystemMode_e mode)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_actuator_state.mode = mode;
        xSemaphoreGive(g_state_mutex);
    }
}

bool Sync_PushCommandWithBrakePriority(const Command_t *cmd)
{
    if (cmd == NULL || g_cmd_queue == NULL) return false;

    /*
     * Safety guarantee: BRAKE commands are pushed to the head of the queue.
     * If the queue is full, the oldest command is evicted to guarantee insertion.
     */
    if (cmd->type == CMD_TYPE_BRAKE) {
        if (uxQueueSpacesAvailable(g_cmd_queue) == 0) {
            Command_t discarded_cmd;
            if (xQueueReceive(g_cmd_queue, &discarded_cmd, 0) == pdTRUE) {
                if (xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
                    s_actuator_state.total_commands_dropped++;
                    xSemaphoreGive(g_state_mutex);
                }
            }
        }
        return (xQueueSendToFront(g_cmd_queue, cmd, 0) == pdPASS);
    } else {
        BaseType_t ret = xQueueSendToBack(g_cmd_queue, cmd, 0);
        if (ret != pdPASS) {
            if (xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
                s_actuator_state.total_commands_dropped++;
                xSemaphoreGive(g_state_mutex);
            }
            return false;
        }
        return true;
    }
}

void UART_Println(const char *msg)
{
    if (msg == NULL) return;
    UART_Printf("%s\r\n", msg);
}

void UART_Printf(const char *fmt, ...)
{
    if (fmt == NULL) return;

    if (g_uart_mutex != NULL) {
        xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    }

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);

    if (g_uart_mutex != NULL) {
        xSemaphoreGive(g_uart_mutex);
    }
}
