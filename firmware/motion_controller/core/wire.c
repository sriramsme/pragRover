// SPDX-License-Identifier: GPL-3.0-only
#include "wire.h"
#include "cJSON.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
static bool bounded_json(const char *line, size_t length) {
    if (length == 0 || length > 1024 || memchr(line, 0, length) != NULL) {
        return false;
    }
    int depth = 0;
    bool quoted = false, escaped = false;
    for (size_t i = 0; i < length; ++i) {
        if (i + 6 <= length && memcmp(line + i, "\\u0000", 6) == 0) {
            return false;
        }
        const char ch = line[i];
        if (quoted) {
            if (escaped) {
                escaped = false;
            } else if (ch == '\\') {
                escaped = true;
            } else if (ch == '"') {
                quoted = false;
            }
        } else if (ch == '"') {
            quoted = true;
        } else if (ch == '{' || ch == '[') {
            if (++depth > 4) {
                return false;
            }
        } else if (ch == '}' || ch == ']') {
            if (--depth < 0) {
                return false;
            }
        }
    }
    return depth == 0 && !quoted;
}
static bool number(const cJSON *object, const char *key, double *value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) {
        return false;
    }
    *value = item->valuedouble;
    return true;
}
static const char *string(const cJSON *object, const char *key) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}
const char *rover_parse_request(const char *line, size_t length, RoverRequest *result) {
    if (!line || !result || !bounded_json(line, length)) {
        return "invalid_request";
    }
    const char *end = NULL;
    /* A private terminator lets cJSON enforce full consumption without reading
     * beyond the caller's explicit input length. */
    char input[1025];
    memcpy(input, line, length);
    input[length] = '\0';
    cJSON *object = cJSON_ParseWithLengthOpts(input, length + 1, &end, true);
    const char *error = NULL;
    RoverRequest candidate = {.sequence = -1};
    if (!cJSON_IsObject(object)) {
        error = "invalid_request";
        goto cleanup;
    }
    double version = 0;
    const char *op = string(object, "op");
    if (!op || !number(object, "version", &version) || version != 1) {
        error = "invalid_request";
        goto cleanup;
    }
    if (strcmp(op, "hello") == 0) {
        candidate.op = ROVER_OP_HELLO;
    } else if (strcmp(op, "status") == 0) {
        candidate.op = ROVER_OP_STATUS;
    } else if (strcmp(op, "arm") == 0) {
        candidate.op = ROVER_OP_ARM;
    } else if (strcmp(op, "reset") == 0) {
        candidate.op = ROVER_OP_RESET;
    } else if (strcmp(op, "stop") == 0) {
        candidate.op = ROVER_OP_STOP;
    } else if (strcmp(op, "drive") == 0) {
        candidate.op = ROVER_OP_DRIVE;
    } else {
        error = "unknown_operation";
        goto cleanup;
    }
    for (const cJSON *a = object->child; a; a = a->next) {
        for (const cJSON *b = a->next; b; b = b->next) {
            if (strcmp(a->string, b->string) == 0) {
                error = "duplicate_field";
                goto cleanup;
            }
        }
        const char *key = a->string;
        bool allowed = strcmp(key, "version") == 0 || strcmp(key, "op") == 0;
        if (candidate.op == ROVER_OP_HELLO) {
            allowed |= strcmp(key, "token") == 0;
        } else {
            allowed |= strcmp(key, "session") == 0 || strcmp(key, "seq") == 0 ||
                       strcmp(key, "ticket") == 0;
            if (candidate.op == ROVER_OP_DRIVE) {
                allowed |= strcmp(key, "left_mps") == 0 || strcmp(key, "right_mps") == 0 ||
                           strcmp(key, "lease_ms") == 0;
            }
        }
        if (!allowed) {
            error = "unknown_field";
            goto cleanup;
        }
    }
    if (candidate.op == ROVER_OP_HELLO) {
        const char *token = string(object, "token");
        if (!token || strlen(token) < 16 || strlen(token) > 128) {
            error = "authentication_failed";
            goto cleanup;
        }
        memcpy(candidate.token, token, strlen(token) + 1);
    } else {
        const char *session = string(object, "session");
        double sequence = 0;
        if (!session || strlen(session) != 32) {
            error = "session_rejected";
            goto cleanup;
        }
        if (!number(object, "seq", &sequence) || sequence < 0 || sequence > 9007199254740991.0 ||
            floor(sequence) != sequence) {
            error = "invalid_sequence";
            goto cleanup;
        }
        memcpy(candidate.session, session, 33);
        candidate.sequence = (int64_t)(sequence);
        const char *ticket = string(object, "ticket");
        if (cJSON_GetObjectItemCaseSensitive(object, "ticket") && !ticket) {
            error = "ticket_rejected";
            goto cleanup;
        }
        if (ticket) {
            if (strlen(ticket) > 32) {
                error = "ticket_rejected";
                goto cleanup;
            }
            memcpy(candidate.ticket, ticket, strlen(ticket) + 1);
        }
        if (candidate.op != ROVER_OP_STOP && candidate.op != ROVER_OP_STATUS &&
            (!ticket || strlen(ticket) != 32)) {
            error = "ticket_rejected";
            goto cleanup;
        }
        if (candidate.op == ROVER_OP_DRIVE) {
            double lease = 0;
            if (!number(object, "left_mps", &candidate.left_mps) ||
                !number(object, "right_mps", &candidate.right_mps) ||
                !number(object, "lease_ms", &lease) || floor(lease) != lease || lease < 1 ||
                lease > 300) {
                error = "invalid_drive";
                goto cleanup;
            }
            candidate.lease_ms = (int)lease;
        }
    }
    *result = candidate;
cleanup:
    cJSON_Delete(object);
    return error;
}
