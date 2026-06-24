/**
 * @file  sil_status.h
 * @brief Status file writer (used by nstar_app_sil) and reader (used by
 *        testSil) for black-box observation of application state.
 *
 * The application is a separate process from the test suite, so the only
 * way the test suite can observe its internal state is through files it
 * writes. This header implements the agreed KEY=VALUE format from
 * sil_common.h's SIL_STATUS_FILE.
 *
 * Write side (nstar_app_sil.c):
 *   Call silStatusWrite() with the current snapshot every time something
 *   observable changes (moduleState transition, rxState transition,
 *   a fault fires, a TX session completes, a frame is received).
 *   The whole file is rewritten each time via a temp-file-then-rename
 *   pattern so a concurrent reader never sees a partial write.
 *
 * Read side (testSil.c):
 *   Call silStatusRead() to get the latest snapshot.
 *   Call silStatusWaitFor() to poll until a specific key reaches an
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
} silStatusKV_t;

typedef struct {
    silStatusKV_t kv[SIL_STATUS_MAX_KEYS];
    int             count;
} silStatus_t;

/** Initialise an empty status snapshot. */
static inline void silStatusInit(silStatus_t *s)
{
    s->count = 0;
}

/** Set (or update) a key in the snapshot. No-op if the table is full. */
static inline void silStatusSet(silStatus_t *s, const char *key,
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
static inline void silStatusSetInt(silStatus_t *s, const char *key,
                                       long val)
{
    char buf[SIL_STATUS_VAL_LEN];
    snprintf(buf, sizeof(buf), "%ld", val);
    silStatusSet(s, key, buf);
}

/**
 * Write the snapshot to SIL_STATUS_FILE atomically.
 * Writes to a temp file then renames over the real path, so a reader
 * polling with stat()/fopen() never observes a half-written file.
 */
static inline void silStatusWrite(const silStatus_t *s)
{
    char path[512], tmpPath[520];
    silPath(path, sizeof(path), SIL_STATUS_FILE);
    snprintf(tmpPath, sizeof(tmpPath), "%s.tmp", path);

    FILE *f = fopen(tmpPath, "w");
    if (!f) return;
    for (int i = 0; i < s->count; i++) {
        fprintf(f, "%s=%s\n", s->kv[i].key, s->kv[i].val);
    }
    fclose(f);
    rename(tmpPath, path);
}

/**
 * Read the current status file into a snapshot.
 * @return 1 on success, 0 if the file does not exist yet or is unreadable.
 */
static inline int silStatusRead(silStatus_t *out)
{
    char path[512];
    silPath(path, sizeof(path), SIL_STATUS_FILE);

    FILE *f = fopen(path, "r");
    if (!f) return 0;

    silStatusInit(out);
    char line[128];
    while (fgets(line, sizeof(line), f) && out->count < SIL_STATUS_MAX_KEYS) {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *val = eq + 1;
        char *nl = strchr(val, '\n');
        if (nl) *nl = '\0';
        silStatusSet(out, line, val);
    }
    fclose(f);
    return 1;
}

/** Look up a key in a snapshot. Returns NULL if not present. */
static inline const char *silStatusGet(const silStatus_t *s,
                                          const char *key)
{
    for (int i = 0; i < s->count; i++) {
        if (strcmp(s->kv[i].key, key) == 0) return s->kv[i].val;
    }
    return NULL;
}

/**
 * Poll the status file until `key` equals `expectedVal`, or timeout.
 * @return 1 if the condition was observed, 0 on timeout.
 */
static inline int silStatusWaitFor(const char *key,
                                       const char *expectedVal,
                                       int timeoutMs)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        silStatus_t snap;
        if (silStatusRead(&snap)) {
            const char *v = silStatusGet(&snap, key);
            if (v && strcmp(v, expectedVal) == 0) return 1;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsedMS = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsedMS >= timeoutMs) return 0;

        struct timespec pollDelay = { 0, 20 * 1000000L };  /* 20 ms */
        nanosleep(&pollDelay, NULL);
    }
}

/**
 * Poll until `key` is present with ANY value different from `notVal`,
 * or timeout. Useful for "wait until this counter changes" style checks
 * where the exact target value isn't known in advance.
 */
static inline int silStatusWaitForChange(const char *key,
                                              const char *notVal,
                                              int timeoutMs)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        silStatus_t snap;
        if (silStatusRead(&snap)) {
            const char *v = silStatusGet(&snap, key);
            if (v && strcmp(v, notVal) != 0) return 1;
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsedMS = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsedMS >= timeoutMs) return 0;

        struct timespec pollDelay = { 0, 20 * 1000000L };
        nanosleep(&pollDelay, NULL);
    }
}

#endif /* SIL_STATUS_H */
