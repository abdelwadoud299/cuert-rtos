/**
 * @file tasks.h
 * @brief Declarations of the four FreeRTOS tasks.
 */

#ifndef CUERT_TASKS_H
#define CUERT_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

void Task_CommandRx(void *pvParameters);
void Task_Actuate(void *pvParameters);
void Task_Watchdog(void *pvParameters);
void Task_Status(void *pvParameters);

extern TaskHandle_t g_handle_command_rx;
extern TaskHandle_t g_handle_actuate;
extern TaskHandle_t g_handle_watchdog;
extern TaskHandle_t g_handle_status;

#ifdef __cplusplus
}
#endif

#endif /* CUERT_TASKS_H */
