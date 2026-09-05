/**
 * @file types.h
 * @brief Core data types and structures for the RTOS Sensor-to-Actuator Node.
 */

#ifndef CUERT_TYPES_H
#define CUERT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Command identifiers supported by the controller.
 */
typedef enum {
    CMD_TYPE_NONE     = 0,
    CMD_TYPE_THROTTLE = 'T',
    CMD_TYPE_STEER    = 'S',
    CMD_TYPE_BRAKE    = 'B',
    CMD_TYPE_PING     = 'P'
} CommandType_e;

/**
 * @brief Operational system modes.
 */
typedef enum {
    SYSTEM_MODE_INIT = 0,
    SYSTEM_MODE_NORMAL,
    SYSTEM_MODE_FAIL_SAFE
} SystemMode_e;

/**
 * @brief Inter-task command message structure (passed by value via FreeRTOS queue).
 */
typedef struct {
    char     type;          /* 'T', 'S', 'B', 'P' */
    int16_t  value;         /* 0..100 (Throttle/Brake) or -100..100 (Steer) */
    uint32_t timestamp_ms;  /* Millisecond timestamp when received */
} Command_t;

/**
 * @brief Complete runtime actuator and vehicle state snapshot.
 */
typedef struct {
    int16_t      throttle_percent;
    int16_t      steer_value;
    int16_t      brake_percent;
    uint8_t      effective_pwm_percent;
    bool         brake_override_active;
    SystemMode_e mode;
    char         last_cmd_type;
    int16_t      last_cmd_value;
    uint32_t     last_cmd_timestamp_ms;
    uint32_t     total_commands_received;
    uint32_t     total_commands_dropped;
} ActuatorState_t;

#ifdef __cplusplus
}
#endif

#endif /* CUERT_TYPES_H */
