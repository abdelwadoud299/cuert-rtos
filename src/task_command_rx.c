/**
 * @file task_command_rx.c
 * @brief FreeRTOS Task 1: COMMAND_RX (Priority 5 - Highest).
 */

#include "tasks.h"
#include "types.h"
#include "config.h"
#include "sync.h"
#include "parser.h"

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_timer.h"

TaskHandle_t g_handle_command_rx = NULL;

void Task_CommandRx(void *pvParameters)
{
    (void)pvParameters;

    char line_buffer[CMD_LINE_BUFFER_MAX_LEN];
    size_t line_index = 0;
    char err_msg[64];

    UART_Println("[COMMAND_RX] Task started. Listening at 115200 baud...");

    while (1) {
        uint8_t rx_byte = 0;
        int bytes_read = uart_read_bytes(UART_NUM_0, &rx_byte, 1, portMAX_DELAY);

        if (bytes_read <= 0) continue;

        if (rx_byte == '\r' || rx_byte == '\n') {
            if (line_index > 0) {
                line_buffer[line_index] = '\0';

                Command_t cmd;
                memset(&cmd, 0, sizeof(Command_t));
                err_msg[0] = '\0';

                ParseResult_e parse_res = Parser_ParseCommandLine(line_buffer, &cmd, err_msg, sizeof(err_msg));

                if (parse_res == PARSE_OK) {
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                    cmd.timestamp_ms = now_ms;

                    /* Refresh watchdog heartbeat */
                    Sync_UpdateLastCommandTimeMs(now_ms);

                    if (cmd.type == CMD_TYPE_PING) {
                        UART_Println("PONG (ACK: PING received)");
                    }

                    bool queued = Sync_PushCommandWithBrakePriority(&cmd);
                    if (!queued && cmd.type != CMD_TYPE_BRAKE) {
                        UART_Printf("[COMMAND_RX] WARNING: Queue full! Dropped %c setpoint\r\n", cmd.type);
                    }
                } else if (parse_res != PARSE_ERR_EMPTY_LINE) {
                    UART_Printf("[COMMAND_RX] PARSER ERROR: %s (input: '%s')\r\n", err_msg, line_buffer);
                }

                line_index = 0;
            }
        } else if (rx_byte == '\b' || rx_byte == 127) {
            if (line_index > 0) line_index--;
        } else {
            if (line_index < sizeof(line_buffer) - 1) {
                line_buffer[line_index++] = (char)rx_byte;
            } else {
                line_buffer[sizeof(line_buffer) - 1] = '\0';
                UART_Printf("[COMMAND_RX] ERROR: Line exceeds %d bytes. Flushed.\r\n", CMD_LINE_BUFFER_MAX_LEN);
                line_index = 0;
            }
        }
    }
}
