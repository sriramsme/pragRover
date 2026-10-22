// SPDX-License-Identifier: GPL-3.0-only
#include "cJSON.h"
#include "wire.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define COMMON                                                                                     \
    "\"version\":1,\"session\":\"0123456789abcdef0123456789abcdef\",\"seq\":1,\"ticket\":"         \
    "\"abcdef0123456789abcdef0123456789\""
#define DRIVE "{" COMMON ",\"op\":\"drive\",\"left_mps\":0.1,\"right_mps\":-0.1,\"lease_ms\":200}"

static void rejected(const char *line) {
    RoverRequest request = {.sequence = 42};
    assert(rover_parse_request(line, strlen(line), &request));
    assert(request.sequence == 42); /* Failure never publishes partial output. */
}

static void rejected_replacement(const char *from, const char *to) {
    char line[1400];
    const char *match = strstr(DRIVE, from);
    assert(match);
    const size_t prefix = (size_t)(match - DRIVE);
    memcpy(line, DRIVE, prefix);
    const int written =
        snprintf(line + prefix, sizeof(line) - prefix, "%s%s", to, match + strlen(from));
    assert(written >= 0 && (size_t)written < sizeof(line) - prefix);
    rejected(line);
}

static size_t allocation_calls, fail_at, outstanding;

static void *test_allocate(size_t size) {
    if (++allocation_calls == fail_at) {
        return NULL;
    }
    void *memory = malloc(size);
    if (memory) {
        ++outstanding;
    }
    return memory;
}

static void test_release(void *memory) {
    if (memory) {
        assert(outstanding > 0);
        --outstanding;
        free(memory);
    }
}

static void allocation_failure_cleanup(const char *line) {
    cJSON_Hooks hooks = {.malloc_fn = test_allocate, .free_fn = test_release};
    cJSON_InitHooks(&hooks);
    for (fail_at = 1; fail_at <= 40; ++fail_at) {
        allocation_calls = 0;
        RoverRequest result = {.sequence = 42};
        const char *error = rover_parse_request(line, strlen(line), &result);
        assert(outstanding == 0);
        if (error) {
            assert(result.sequence == 42);
        }
    }
    cJSON_InitHooks(NULL);
}

int main(void) {
    RoverRequest request;
    const char hello[] = "{\"version\":1,\"op\":\"hello\",\"token\":\"0123456789abcdef\"}";
    assert(!rover_parse_request(hello, strlen(hello), &request));
    assert(request.op == ROVER_OP_HELLO && strcmp(request.token, "0123456789abcdef") == 0);
    assert(!rover_parse_request(DRIVE, strlen(DRIVE), &request));
    assert(request.op == ROVER_OP_DRIVE && request.lease_ms == 200 && request.left_mps == 0.1 &&
           request.right_mps == -0.1);
    const char status[] = "{" COMMON ",\"op\":\"status\"}";
    const char stop[] = "{\"version\":1,\"op\":\"stop\",\"session\":"
                        "\"0123456789abcdef0123456789abcdef\",\"seq\":2}";
    assert(!rover_parse_request(status, strlen(status), &request));
    assert(!rover_parse_request(stop, strlen(stop), &request));
    const char *invalid[] = {
        "{\"version\":true,\"op\":\"hello\",\"token\":\"0123456789abcdef\"}",
        "{\"version\":2,\"op\":\"hello\",\"token\":\"0123456789abcdef\"}",
        "{\"version\":1,\"op\":\"hello\",\"token\":\"0123456789abcdef\",\"token\":"
        "\"anothercredential\"}",
        "{\"version\":1,\"op\":\"hello\",\"token\":\"0123456789abcdef\",\"extra\":0}",
        "{\"version\":1,\"op\":\"hello\",\"token\":\"short\"}",
        "{\"version\":1,\"op\":\"hello\",\"token\":12345}",
        "{\"version\":1,\"op\":\"hello\\u0000stop\",\"token\":\"0123456789abcdef\"}",
        "[" DRIVE "]",
        DRIVE "trailing",
        "{\"version\":1,\"op\":\"hello\",\"token\":[[[[[0]]]]]}",
        "{" COMMON ",\"op\":\"status\",\"ticket\":null}"};
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        rejected(invalid[i]);
    }
    const char *replacements[][2] = {{"\"seq\":1", "\"seq\":true"},
                                     {"\"seq\":1", "\"seq\":1.1"},
                                     {"\"seq\":1", "\"seq\":-1"},
                                     {"\"seq\":1", "\"seq\":9007199254740992"},
                                     {"\"lease_ms\":200", "\"lease_ms\":301"},
                                     {"\"lease_ms\":200", "\"lease_ms\":0"},
                                     {"\"lease_ms\":200", "\"lease_ms\":1.5"},
                                     {"\"left_mps\":0.1", "\"left_mps\":1e999"},
                                     {"\"left_mps\":0.1", "\"left_mps\":null"},
                                     {"\"left_mps\":0.1", "\"left_mps\":\"0.1\""},
                                     {"\"left_mps\":0.1", "\"left_mps\":0.1,\"left_mps\":0.2"},
                                     {"abcdef0123456789abcdef0123456789", "bad_ticket"},
                                     {"0123456789abcdef0123456789abcdef", "bad_session"},
                                     {"\"op\":\"drive\"", "\"op\":\"launch\""}};
    for (size_t i = 0; i < sizeof(replacements) / sizeof(replacements[0]); ++i) {
        rejected_replacement(replacements[i][0], replacements[i][1]);
    }
    char oversized[1101];
    memset(oversized, ' ', sizeof(oversized));
    assert(rover_parse_request(oversized, sizeof(oversized), &request));
    char unterminated[sizeof(hello) - 1];
    memcpy(unterminated, hello, sizeof(unterminated));
    assert(!rover_parse_request(unterminated, sizeof(unterminated), &request));
    char embedded_null[sizeof(hello)];
    memcpy(embedded_null, hello, sizeof(hello));
    embedded_null[10] = '\0';
    assert(rover_parse_request(embedded_null, sizeof(embedded_null) - 1, &request));
    const char long_ticket[] =
        "{\"version\":1,\"op\":\"status\",\"session\":\"0123456789abcdef0123456789abcdef\",\"seq\":"
        "1,\"ticket\":\"123456789012345678901234567890123\"}";
    rejected(long_ticket);
    char maximum_line[1024];
    memset(maximum_line, ' ', sizeof(maximum_line));
    memcpy(maximum_line, hello, strlen(hello));
    assert(!rover_parse_request(maximum_line, sizeof(maximum_line), &request));
    char token[130], token_request[256];
    memset(token, 'x', 128);
    token[128] = '\0';
    snprintf(token_request, sizeof(token_request),
             "{\"version\":1,\"op\":\"hello\",\"token\":\"%s\"}", token);
    assert(!rover_parse_request(token_request, strlen(token_request), &request));
    assert(strlen(request.token) == 128);
    token[128] = 'x';
    token[129] = '\0';
    snprintf(token_request, sizeof(token_request),
             "{\"version\":1,\"op\":\"hello\",\"token\":\"%s\"}", token);
    rejected(token_request);
    allocation_failure_cleanup(hello);
    allocation_failure_cleanup(DRIVE);
    puts("Strict C firmware wire parser cases passed");
    return 0;
}
