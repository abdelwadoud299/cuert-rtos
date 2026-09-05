/**
 * @file cuert_rtos_controller.ino
 * @brief Real-Time Vehicle Actuation & Fail-Safe Supervisor Firmware
 * @target ESP32-CAM (AI Thinker) with CAM-MB baseboard
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ========================================================================== */
/*                             SYSTEM CONFIGURATION                           */
/* ========================================================================== */

/* ESP32-CAM: Onboard red indicator LED on GPIO 33 (Active-LOW: 0V = ON) */
#define LED_PIN                     33
#define LED_ACTIVE_LOW              1

#define PWM_FREQ                    5000    /* 5 kHz PWM frequency */
#define PWM_RESOLUTION_BITS         10      /* 10-bit resolution (0..1023) */
#define PWM_MAX_DUTY                1023
#define PWM_CHANNEL                 0

#define SERIAL_BAUD                 115200
#define CMD_QUEUE_LEN               10

#define WATCHDOG_TIMEOUT_MS         500     /* 500 ms communication loss timeout */
#define WATCHDOG_CHECK_MS           50      /* 50 ms supervision cycle */
#define SAFE_IDLE_BLINK_MS          250     /* 2 Hz safe-idle blink */
#define STATUS_PERIOD_MS            1000    /* 1 Hz status telemetry period */

/* Task Priorities */
#define PRIORITY_COMMAND_RX         5       /* Highest: UART command ingestion */
#define PRIORITY_ACTUATE            4       /* Medium: PWM drive & safety interlock */
#define PRIORITY_WATCHDOG           3       /* Low: Link supervision & fail-safe */
#define PRIORITY_STATUS             2       /* Lowest: Telemetry logger */

/* ========================================================================== */
/*                                DATA TYPES                                  */
/* ========================================================================== */

typedef enum {
    MODE_INIT = 0,
    MODE_NORMAL,
    MODE_FAIL_SAFE
} SystemMode_e;

typedef struct {
    char     type;          /* 'T', 'S', 'B', 'P' */
    int16_t  value;         /* 0..100 (Throttle/Brake) or -100..100 (Steer) */
    uint32_t timestamp_ms;  /* Reception timestamp */
} Command_t;

typedef struct {
    int16_t      throttle_percent;
    int16_t      steer_value;
    int16_t      brake_percent;
    uint8_t      effective_pwm;
    bool         brake_override;
    SystemMode_e mode;
    char         last_cmd_type;
    int16_t      last_cmd_val;
    uint32_t     last_cmd_time_ms;
} VehicleState_t;

/* ========================================================================== */
/*                          SYNCHRONIZATION & STATE                           */
/* ========================================================================== */

static QueueHandle_t     g_cmd_queue   = NULL;
static SemaphoreHandle_t g_state_mutex = NULL;
static SemaphoreHandle_t g_uart_mutex  = NULL;

static volatile uint32_t s_last_cmd_time_ms = 0;
static VehicleState_t    s_state = {0, 0, 0, 0, false, MODE_INIT, '-', 0, 0};

void SafePrint(const String &str) {
    if (g_uart_mutex != NULL) {
        xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    }
    Serial.print(str);
    if (g_uart_mutex != NULL) {
        xSemaphoreGive(g_uart_mutex);
    }
}

void SafePrintln(const String &str) {
    SafePrint(str + "\r\n");
}

void SetPwmPercent(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint32_t duty = ((uint32_t)percent * PWM_MAX_DUTY) / 100UL;

#if LED_ACTIVE_LOW
    duty = PWM_MAX_DUTY - duty;
#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(LED_PIN, duty);
#else
    ledcWrite(PWM_CHANNEL, duty);
#endif
}

/* ========================================================================== */
/*                         TASK 1: COMMAND_RX (PRIORITY 5)                    */
/* ========================================================================== */

void Task_CommandRx(void *pvParameters) {
    (void)pvParameters;
    String line = "";

    SafePrintln("[COMMAND_RX] Task online. Listening at 115200 baud.");

    while (1) {
        if (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\r' || c == '\n') {
                line.trim();
                if (line.length() > 0) {
                    Command_t cmd;
                    memset(&cmd, 0, sizeof(Command_t));
                    bool valid = false;

                    if (line.equals("PING")) {
                        cmd.type = 'P';
                        cmd.value = 0;
                        valid = true;
                        SafePrintln("PONG (ACK: PING)");
                    } else if (line.startsWith("THROTTLE ") || line.startsWith("STEER ") || line.startsWith("BRAKE ")) {
                        int spaceIdx = line.indexOf(' ');
                        String cmdStr = line.substring(0, spaceIdx);
                        String valStr = line.substring(spaceIdx + 1);
                        valStr.trim();

                        bool isNumeric = (valStr.length() > 0);
                        int startIdx = 0;
                        if (valStr.charAt(0) == '-') {
                            if (valStr.length() == 1) isNumeric = false;
                            startIdx = 1;
                        }
                        for (int i = startIdx; i < valStr.length(); i++) {
                            if (!isDigit(valStr.charAt(i))) {
                                isNumeric = false;
                                break;
                            }
                        }

                        if (!isNumeric) {
                            SafePrintln("[COMMAND_RX] PARSER ERROR: Invalid number '" + valStr + "'");
                        } else {
                            long val = valStr.toInt();
                            if (cmdStr.equals("THROTTLE")) {
                                if (val >= 0 && val <= 100) {
                                    cmd.type = 'T'; cmd.value = (int16_t)val; valid = true;
                                } else {
                                    SafePrintln("[COMMAND_RX] ERROR: THROTTLE out of bounds [0..100]");
                                }
                            } else if (cmdStr.equals("STEER")) {
                                if (val >= -100 && val <= 100) {
                                    cmd.type = 'S'; cmd.value = (int16_t)val; valid = true;
                                } else {
                                    SafePrintln("[COMMAND_RX] ERROR: STEER out of bounds [-100..100]");
                                }
                            } else if (cmdStr.equals("BRAKE")) {
                                if (val >= 0 && val <= 100) {
                                    cmd.type = 'B'; cmd.value = (int16_t)val; valid = true;
                                } else {
                                    SafePrintln("[COMMAND_RX] ERROR: BRAKE out of bounds [0..100]");
                                }
                            }
                        }
                    } else {
                        SafePrintln("[COMMAND_RX] PARSER ERROR: Unknown command '" + line + "'");
                    }

                    if (valid) {
                        uint32_t now = millis();
                        cmd.timestamp_ms = now;
                        s_last_cmd_time_ms = now;

                        /* BRAKE Priority Handling */
                        if (cmd.type == 'B') {
                            if (uxQueueSpacesAvailable(g_cmd_queue) == 0) {
                                Command_t discarded;
                                xQueueReceive(g_cmd_queue, &discarded, 0);
                            }
                            xQueueSendToFront(g_cmd_queue, &cmd, 0);
                        } else {
                            if (xQueueSendToBack(g_cmd_queue, &cmd, 0) != pdPASS) {
                                SafePrintln("[COMMAND_RX] WARNING: Queue full! Non-brake command dropped.");
                            }
                        }
                    }

                    line = "";
                }
            } else if (c >= 32 && c <= 126) {
                line += c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

/* ========================================================================== */
/*                         TASK 2: ACTUATE (PRIORITY 4)                       */
/* ========================================================================== */

void Task_Actuate(void *pvParameters) {
    (void)pvParameters;
    Command_t rx_cmd;
    int16_t throttle = 0;
    int16_t steer = 0;
    int16_t brake = 0;
    uint8_t effective_pwm = 0;
    bool brake_override = false;

    SafePrintln("[ACTUATE] Task online. Ready to drive PWM.");

    while (1) {
        if (xQueueReceive(g_cmd_queue, &rx_cmd, portMAX_DELAY) == pdTRUE) {
            if (rx_cmd.type == 'T') throttle = rx_cmd.value;
            else if (rx_cmd.type == 'S') steer = rx_cmd.value;
            else if (rx_cmd.type == 'B') brake = rx_cmd.value;

            /* Brake Override Interlock */
            if (brake > 0) {
                brake_override = true;
                effective_pwm = 0;
            } else {
                brake_override = false;
                effective_pwm = (uint8_t)throttle;
            }

            if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (s_state.mode == MODE_FAIL_SAFE) {
                    s_state.mode = MODE_NORMAL;
                }
                s_state.throttle_percent = throttle;
                s_state.steer_value = steer;
                s_state.brake_percent = brake;
                s_state.effective_pwm = effective_pwm;
                s_state.brake_override = brake_override;
                s_state.last_cmd_type = rx_cmd.type;
                s_state.last_cmd_val = rx_cmd.value;
                s_state.last_cmd_time_ms = rx_cmd.timestamp_ms;
                xSemaphoreGive(g_state_mutex);
            }

            SetPwmPercent(effective_pwm);

            char buf[128];
            snprintf(buf, sizeof(buf), 
                     "[ACTUATE] Executed: Cmd='%c' Val=%4d | State: Throttle=%3d%% Steer=%4d Brake=%3d%% | PWM=%3d%% %s",
                     rx_cmd.type, rx_cmd.value, throttle, steer, brake, effective_pwm,
                     brake_override ? "[BRAKE OVERRIDE]" : "");
            SafePrintln(buf);
        }
    }
}

/* ========================================================================== */
/*                         TASK 3: WATCHDOG (PRIORITY 3)                      */
/* ========================================================================== */

void Task_Watchdog(void *pvParameters) {
    (void)pvParameters;
    bool in_failsafe = false;
    bool led_blink = false;
    uint32_t last_toggle = 0;

    SafePrintln("[WATCHDOG] Task online. Supervising link (500ms timeout).");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_MS));
        uint32_t now = millis();
        uint32_t elapsed = (now >= s_last_cmd_time_ms) ? (now - s_last_cmd_time_ms) : 0;

        if (elapsed >= WATCHDOG_TIMEOUT_MS) {
            if (!in_failsafe) {
                in_failsafe = true;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_state.mode = MODE_FAIL_SAFE;
                    xSemaphoreGive(g_state_mutex);
                }
                SafePrintln("LINK LOST , failing safe");
                SetPwmPercent(0);
                led_blink = false;
                last_toggle = now;
            }

            if (now - last_toggle >= SAFE_IDLE_BLINK_MS) {
                led_blink = !led_blink;
                SetPwmPercent(led_blink ? 100 : 0);
                last_toggle = now;
            }
        } else {
            if (in_failsafe) {
                in_failsafe = false;
                SafePrintln("[WATCHDOG] LINK RESTORED: Resuming normal actuation");

                uint8_t restored_pwm = 0;
                if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_state.mode = MODE_NORMAL;
                    restored_pwm = s_state.effective_pwm;
                    xSemaphoreGive(g_state_mutex);
                }
                SetPwmPercent(restored_pwm);
            }
        }
    }
}

/* ========================================================================== */
/*                         TASK 4: STATUS (PRIORITY 2)                        */
/* ========================================================================== */

void Task_Status(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(STATUS_PERIOD_MS);

    SafePrintln("[STATUS] Task online. Telemetry at 1 Hz.");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t now = millis();
        static VehicleState_t snap = {0};
        if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            snap = s_state;
            xSemaphoreGive(g_state_mutex);
        }

        uint32_t age = (now >= s_last_cmd_time_ms) ? (now - s_last_cmd_time_ms) : 0;
        const char *mode_str = (snap.mode == MODE_FAIL_SAFE) ? "FAIL-SAFE" : "NORMAL";

        char buf[160];
        snprintf(buf, sizeof(buf),
                 "[STATUS] Uptime: %4lus | LastCmd: %c=%-4d (%4lums ago) | Output: PWM=%3d%% (T:%3d%% S:%-4d B:%3d%%) | Mode: %s",
                 now / 1000UL, snap.last_cmd_type, snap.last_cmd_val, age,
                 snap.effective_pwm, snap.throttle_percent, snap.steer_value, snap.brake_percent, mode_str);
        SafePrintln(buf);
    }
}

/* ========================================================================== */
/*                               ARDUINO SETUP                                */
/* ========================================================================== */

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 2000);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION_BITS);
    ledcAttachPin(LED_PIN, PWM_CHANNEL);
#endif
    SetPwmPercent(0);

    g_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(Command_t));
    g_state_mutex = xSemaphoreCreateMutex();
    g_uart_mutex = xSemaphoreCreateMutex();
    s_last_cmd_time_ms = millis();

    SafePrintln("\r\n========================================================");
    SafePrintln("  REAL-TIME ACTUATION & FAIL-SAFE SUPERVISOR (ESP32)   ");
    SafePrintln("      FreeRTOS Autonomous Automotive Gateway Node       ");
    SafePrintln("========================================================\r\n");

    xTaskCreatePinnedToCore(Task_CommandRx, "CMD_RX",   4096, NULL, PRIORITY_COMMAND_RX, NULL, 1);
    xTaskCreatePinnedToCore(Task_Actuate,   "ACTUATE",  3072, NULL, PRIORITY_ACTUATE,    NULL, 1);
    xTaskCreatePinnedToCore(Task_Watchdog,  "WATCHDOG", 3072, NULL, PRIORITY_WATCHDOG,  NULL, 1);
    xTaskCreatePinnedToCore(Task_Status,    "STATUS",   3072, NULL, PRIORITY_STATUS,    NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
