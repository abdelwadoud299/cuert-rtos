/**
 * @file config.h
 * @brief System hardware pinouts, task priorities, queue depths, and timing thresholds.
 */

#ifndef CUERT_CONFIG_H
#define CUERT_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Target Board Selection: 1 = ESP32-CAM (CAM-MB), 0 = Standard ESP32 DevKit V1 */
#define BOARD_IS_ESP32_CAM              (1)

#if BOARD_IS_ESP32_CAM
  #define ACTUATOR_LED_GPIO             (33)  /* Small onboard red LED (Active-LOW) */
  #define ACTUATOR_LED_ACTIVE_LEVEL     (0)   /* 0 = Active-LOW */
#else
  #define ACTUATOR_LED_GPIO             (2)   /* Standard DevKit onboard blue LED */
  #define ACTUATOR_LED_ACTIVE_LEVEL     (1)   /* 1 = Active-HIGH */
#endif

/* Hardware PWM Configuration (LEDC Peripheral) */
#define ACTUATOR_PWM_FREQ_HZ            (5000)
#define ACTUATOR_PWM_RESOLUTION_BITS    (10)  /* 10-bit: 0 .. 1023 */
#define ACTUATOR_PWM_MAX_DUTY           ((1 << ACTUATOR_PWM_RESOLUTION_BITS) - 1)

/* Serial Interface */
#define SERIAL_BAUD_RATE                (115200)
#define CMD_LINE_BUFFER_MAX_LEN         (64)

/* RTOS Queue */
#define COMMAND_QUEUE_LENGTH            (10)

/* RTOS Task Priorities (Higher number = Higher priority) */
#define TASK_PRIORITY_COMMAND_RX        (5)   /* Highest: Drains UART FIFO */
#define TASK_PRIORITY_ACTUATE           (4)   /* Medium: Controls PWM and safety override */
#define TASK_PRIORITY_WATCHDOG          (3)   /* Low: Evaluates 500ms link timeout */
#define TASK_PRIORITY_STATUS            (2)   /* Lowest: Background telemetry logger */

/* Task Stack Allocations (bytes) */
#define STACK_SIZE_COMMAND_RX           (4096)
#define STACK_SIZE_ACTUATE              (3072)
#define STACK_SIZE_WATCHDOG             (3072)
#define STACK_SIZE_STATUS               (3072)

/* Safety & Timing Deadlines */
#define WATCHDOG_TIMEOUT_MS             (500)
#define WATCHDOG_CHECK_PERIOD_MS        (50)
#define SAFE_IDLE_BLINK_HALF_PERIOD_MS  (250) /* 2 Hz blink */
#define STATUS_PERIOD_MS                (1000)

#ifdef __cplusplus
}
#endif

#endif /* CUERT_CONFIG_H */
