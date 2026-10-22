// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stddef.h>
#include <stdint.h>

typedef enum {
    ROVER_OP_HELLO,
    ROVER_OP_STATUS,
    ROVER_OP_ARM,
    ROVER_OP_RESET,
    ROVER_OP_STOP,
    ROVER_OP_DRIVE
} RoverOperation;
typedef struct {
    RoverOperation op;
    char token[129], session[33], ticket[33];
    int64_t sequence;
    double left_mps, right_mps;
    int lease_ms;
} RoverRequest;
/* Input length excludes any terminator. Output changes only after success.
 * Strings are copied into caller-owned bounded storage, never parser pointers. */
const char *rover_parse_request(const char *line, size_t length, RoverRequest *result);
