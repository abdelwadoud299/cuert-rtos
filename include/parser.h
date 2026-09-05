/**
 * @file parser.h
 * @brief ASCII command parser and input validator.
 */

#ifndef CUERT_PARSER_H
#define CUERT_PARSER_H

#include "types.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PARSE_OK = 0,
    PARSE_ERR_EMPTY_LINE,
    PARSE_ERR_UNKNOWN_CMD,
    PARSE_ERR_MISSING_VALUE,
    PARSE_ERR_INVALID_NUMBER,
    PARSE_ERR_OUT_OF_RANGE,
    PARSE_ERR_TRAILING_GARBAGE
} ParseResult_e;

ParseResult_e Parser_ParseCommandLine(const char *line_str, 
                                      Command_t *out_cmd,
                                      char *out_err_msg,
                                      size_t err_msg_len);

const char* Parser_ResultToString(ParseResult_e res);

#ifdef __cplusplus
}
#endif

#endif /* CUERT_PARSER_H */
