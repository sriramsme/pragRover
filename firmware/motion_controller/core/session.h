// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Receiver-clock gate; the transport owns opaque ticket/session strings. */
typedef struct {
    int64_t last_sequence, expires;
    bool used;
} RoverSessionGate;
static inline void rover_session_reset(RoverSessionGate *gate) {
    *gate = (RoverSessionGate){.last_sequence = -1, .used = true};
}
static inline void rover_session_issue(RoverSessionGate *gate, int64_t now) {
    gate->expires = now + 250000;
    gate->used = false;
}
static inline const char *rover_session_authorize(RoverSessionGate *gate, int64_t sequence,
                                                  int64_t now, bool ticket_matches, bool stop,
                                                  bool status) {
    if (sequence < 0 || sequence > 9007199254740991LL || sequence <= gate->last_sequence) {
        return "sequence_rejected";
    }
    if (!stop && !status && (!ticket_matches || gate->used || now >= gate->expires)) {
        return "ticket_rejected";
    }
    gate->last_sequence = sequence;
    if (!status) {
        gate->used = true;
    }
    return NULL;
}
