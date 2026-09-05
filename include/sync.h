/**
 * @file sync.h
 * @brief Thread synchronization, shared state, and safe queue ingestion interfaces.
 */

#ifndef CUERT_SYNC_H
#define CUERT_SYNC_H

#include "types.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t     g_cmd_queue;
extern SemaphoreHandle_t g_state_mutex;
extern SemaphoreHandle_t g_uart_mutex;

BaseType_t Sync_Init(void);

void     Sync_UpdateLastCommandTimeMs(uint32_t now_ms);
uint32_t Sync_GetLastCommandTimeMs(void);

bool Sync_GetStateSnapshot(ActuatorState_t *out_state);
void Sync_UpdateActuatorState(int16_t throttle, int16_t steer, int16_t brake,
                             uint8_t effective_pwm, bool brake_override,
                             char last_cmd_type, int16_t last_cmd_val,
                             uint32_t last_cmd_timestamp);
void Sync_SetSystemMode(SystemMode_e mode);

bool Sync_PushCommandWithBrakePriority(const Command_t *cmd);

void UART_Println(const char *msg);
void UART_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* CUERT_SYNC_H */
