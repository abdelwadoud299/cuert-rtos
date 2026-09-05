/**
 * @file actuator.h
 * @brief Hardware PWM driver for the actuator LED.
 */

#ifndef CUERT_ACTUATOR_H
#define CUERT_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Actuator_Init(void);
void Actuator_SetOutput(uint8_t percent);
void Actuator_SetDigitalLevel(bool is_on);

#ifdef __cplusplus
}
#endif

#endif /* CUERT_ACTUATOR_H */
