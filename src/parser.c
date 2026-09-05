/**
 * @file parser.c
 * @brief Reentrant ASCII command parser and input validator.
 */

#include "parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static const char* skip_whitespace(const char *p)
{
    while (*p != '\0' && isspace((unsigned char)*p)) {
        p++;
    }
    return p;
}

const char* Parser_ResultToString(ParseResult_e res)
{
    switch (res) {
        case PARSE_OK:                  return "OK";
        case PARSE_ERR_EMPTY_LINE:      return "Empty line";
        case PARSE_ERR_UNKNOWN_CMD:     return "Unknown command";
        case PARSE_ERR_MISSING_VALUE:   return "Missing required parameter";
        case PARSE_ERR_INVALID_NUMBER:  return "Invalid non-numeric value";
        case PARSE_ERR_OUT_OF_RANGE:    return "Value out of allowable range";
        case PARSE_ERR_TRAILING_GARBAGE:return "Unexpected trailing characters";
        default:                        return "Parser error";
    }
}

ParseResult_e Parser_ParseCommandLine(const char *line_str, 
                                      Command_t *out_cmd,
                                      char *out_err_msg,
                                      size_t err_msg_len)
{
    if (line_str == NULL || out_cmd == NULL) {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "Null pointer argument");
        }
        return PARSE_ERR_UNKNOWN_CMD;
    }

    out_cmd->type = CMD_TYPE_NONE;
    out_cmd->value = 0;
    out_cmd->timestamp_ms = 0;

    const char *p = skip_whitespace(line_str);
    if (*p == '\0') {
        return PARSE_ERR_EMPTY_LINE;
    }

    char cmd_token[32];
    size_t token_len = 0;
    while (*p != '\0' && !isspace((unsigned char)*p) && token_len < sizeof(cmd_token) - 1) {
        cmd_token[token_len++] = *p++;
    }
    cmd_token[token_len] = '\0';

    if (strcmp(cmd_token, "PING") == 0) {
        p = skip_whitespace(p);
        if (*p != '\0') {
            if (out_err_msg && err_msg_len > 0) {
                snprintf(out_err_msg, err_msg_len, "PING does not accept parameters");
            }
            return PARSE_ERR_TRAILING_GARBAGE;
        }
        out_cmd->type = CMD_TYPE_PING;
        out_cmd->value = 0;
        return PARSE_OK;
    }

    char expected_type = CMD_TYPE_NONE;
    int16_t min_val = 0;
    int16_t max_val = 0;

    if (strcmp(cmd_token, "THROTTLE") == 0) {
        expected_type = CMD_TYPE_THROTTLE;
        min_val = 0;
        max_val = 100;
    } else if (strcmp(cmd_token, "STEER") == 0) {
        expected_type = CMD_TYPE_STEER;
        min_val = -100;
        max_val = 100;
    } else if (strcmp(cmd_token, "BRAKE") == 0) {
        expected_type = CMD_TYPE_BRAKE;
        min_val = 0;
        max_val = 100;
    } else {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "Unknown command '%s'", cmd_token);
        }
        return PARSE_ERR_UNKNOWN_CMD;
    }

    p = skip_whitespace(p);
    if (*p == '\0') {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "%s requires a value [%d..%d]", cmd_token, min_val, max_val);
        }
        return PARSE_ERR_MISSING_VALUE;
    }

    char *endptr = NULL;
    long parsed_val = strtol(p, &endptr, 10);

    if (endptr == p) {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "Expected number but found '%s'", p);
        }
        return PARSE_ERR_INVALID_NUMBER;
    }

    const char *trailing = skip_whitespace(endptr);
    if (*trailing != '\0') {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "Invalid characters '%s' after value", endptr);
        }
        return PARSE_ERR_TRAILING_GARBAGE;
    }

    if (parsed_val < min_val || parsed_val > max_val) {
        if (out_err_msg && err_msg_len > 0) {
            snprintf(out_err_msg, err_msg_len, "Value %ld out of bounds [%d..%d]", parsed_val, min_val, max_val);
        }
        return PARSE_ERR_OUT_OF_RANGE;
    }

    out_cmd->type = expected_type;
    out_cmd->value = (int16_t)parsed_val;
    return PARSE_OK;
}
