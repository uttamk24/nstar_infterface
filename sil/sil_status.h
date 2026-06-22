/**
 * @file  sil_status.h
 * @brief Status file writer (used by nstar_app_sil) and reader (used by
 *        test_sil) for black-box observation of application state.
 *
 * The application is a separate process from the test suite, so the only
 * way the test suite can observe its internal state is through files it
 * writes. This header implements the agreed KEY=VALUE format from
 * sil_common.h's SIL_STATUS_FILE.
 *
 * Write side (nstar_app_sil.c):
 *   Call sil_status_write() with the current snapshot every time something
 *   observable changes (module_state transition, rx_state transition,
 *   a fault fires, a TX session completes, a frame is received).
 *   The whole file is rewritten each time via a temp-file-then-rename
 *   pattern so a concurrent reader never sees a partial write.
 *
 * Read side (test_sil.c):
 *   Call sil_status_read() to get the latest snapshot.
 *   Call sil_status_wait_for() to poll until a specific key reaches an
 *   expected value or a timeout expires — this is the SIL equivalent of
 *   the in-process pthread_cond_t wait used in the unit tests.
 */

#ifndef SIL_STATUS_H
#define SIL_STATUS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "sil_common.h"

#define SIL_STATUS_MAX_KEYS   16
#define SIL_STATUS_KEY_LEN    32
#define SIL_STATUS_VAL_LEN    64

typedef struct {
    char key[SIL_STATUS_KEY_LEN];
    char val[SIL_STATUS_VAL_LEN];
} sil_status_kv_t;

typedef struct {
    sil_status_kv_t kv[SIL_STATUS_MAX_KEYS];
    int             count;
} sil_status_t;

/** Initialise an empty status snapshot. */
static inline void sil_status_init(sil_status_t *s)
{
    s->count = 0;
}

/** Set (or update) a key in the snapshot. No-op if the table is full. */
static inline void sil_status_set(sil_status_t *s, const char *key,
                                   const char *val)
{
    for (int i = 0; i < s->count; i++) {
        if (strcmp(s->kv[i].key, key) == 0) {
            snprintf(s->kv[i].val, SIL_STATUS_VAL_LEN, "%s", val);
            return;
        }
    }
    if (s->count >= SIL_STATUS_MAX_KEYS) return;
    snprintf(s->kv[s->count].key, SIL_STATUS_KEY_LEN, "%s", key);
    snprintf(s->kv[s->count].val, SIL_STATUS_VAL_LEN, "%s", val);
    s->count++;
}

/** Convenience: set an integer-valued key. */
static inline void sil_status_set_int(sil_status_t *s, const char *key,
                                       long val)
{
    char buf[SIL_STATUS_VAL_LEN];
    snprintf(buf, sizeof(buf), "%ld", val);
    sil_status_set(s, key, buf);
}

/**
 * Write the snapshot to SIL_STATUS_FILE atomically.
 * Writes to a temp file then renames over the real path, so a reader
 * polling with stat()/fopen() never observes a half-written file.
 */
static inline void sil_status_write(const sil_status_t *s)
{
    char path[512], tmp_path[520];
    sil_path(path, sizeof(path), SIL_STATUS_FILE);
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    FILE *f = fopen(tmp_path, "w");
    if (!f) return;
    for (int i = 0; i < s->count; i++) {
        fprintf(f, "%s=%s\n", s->kv[i].key, s->kv[i].val);
    }
    fclose(f);
    rename(tmp_path, path);
}

/**
 * Read the current status file into a snapshot.
 * @return 1 on success, 0 if the file does not exist yet or is unreadable.
 */
static inline int sil_status_read(sil_status_t *out)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_STATUS_FILE);

    FILE *f = fopen(path, "r");
    if (!f) return 0;

    sil_status_init(out);
    char line[128];
    while (fgets(line, sizeof(line), f) && out->count < SIL_STATUS_MAX_KEYS) {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *val = eq + 1;
        char *nl = strchr(val, '\n');
        if (nl) *nl = '\0';
        sil_status_set(out, line, val);
    }
    fclose(f);
    return 1;
}

/** Look up a key in a snapshot. Returns NULL if not present. */
static inline const char *sil_status_get(const sil_status_t *s,
                                          const char *key)
{
    for (int i = 0; i < s->count; i++) {
        if (strcmp(s->kv[i].key, key) == 0) return s->kv[i].val;
    }
    return NULL;
}

/**
 * Poll the status file until `key` equals `expected_val`, or timeout.
 * @return 1 if the condition was observed, 0 on timeout.
 */
static inline int sil_status_wait_for(const char *key,
                                       const char *expected_val,
                                       int timeout_ms)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        sil_status_t snap;
        if (sil_status_read(&snap)) {
            const char *v = sil_status_get(&snap, key);
            if (v && strcmp(v, expected_val) == 0) return 1;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= timeout_ms) return 0;

        struct timespec poll_delay = { 0, 20 * 1000000L };  /* 20 ms */
        nanosleep(&poll_delay, NULL);
    }
}

/**
 * Poll until `key` is present with ANY value different from `not_val`,
 * or timeout. Useful for "wait until this counter changes" style checks
 * where the exact target value isn't known in advance.
 */
static inline int sil_status_wait_for_change(const char *key,
                                              const char *not_val,
                                              int timeout_ms)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        sil_status_t snap;
        if (sil_status_read(&snap)) {
            const char *v = sil_status_get(&snap, key);
            if (v && strcmp(v, not_val) != 0) return 1;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= timeout_ms) return 0;

        struct timespec poll_delay = { 0, 20 * 1000000L };
        nanosleep(&poll_delay, NULL);
    }
}

#endif /* SIL_STATUS_H */
