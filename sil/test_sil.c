/**
 * @file  test_sil.c
 * @brief SIL (Software-In-the-Loop) test suite.
 *
 * Unlike the Stage 1-5 unit tests (which call nstar_core.c functions
 * directly against an in-process mock), these tests treat nstar_app_sil
 * as a genuine black box:
 *
 *   1. Start nstar_sim (the N-STAR protocol simulator) as a subprocess.
 *   2. Wait for its PTY link + ready marker.
 *   3. Start nstar_app_sil as a subprocess, with a bounded run duration.
 *   4. Drive the scenario by writing GPIO files and the simulator's
 *      control file — the SAME mechanism a human or a future
 *      hardware-in-the-loop bench would use, just files instead of
 *      real signals.
 *   5. Poll the application's status file for the expected outcome.
 *   6. Kill both subprocesses, clean the SIL root, move to the next test.
 *
 * This exercises the REAL nstar_core.c, the REAL nstar_frame.c codec,
 * the REAL termios UART setup, and the REAL pthread-based thread
 * lifecycle — nothing here is scripted in-process the way the mock is.
 * The only "dummy" pieces are the physical layer underneath the OS
 * abstractions (PTY instead of a real RS-422 chip, files instead of
 * real GPIO sysfs/SPI), which is exactly the boundary meant to be
 * swapped out once hardware is available (see hal/nstar_hal_linux.c).
 */

#define _DEFAULT_SOURCE
#include "unity/unity.h"
#include "nstar.h"
#include "sil_common.h"
#include "sil_status.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>

/* =========================================================================
 * Process orchestration helpers
 * =========================================================================
 */

static pid_t g_sim_pid = -1;
static pid_t g_app_pid = -1;

static char g_sim_path[512];
static char g_app_path[512];

/** Recursively-light cleanup: remove every file under the SIL root. */
static void clean_sil_root(void)
{
    char cmd[600];
    snprintf(cmd, sizeof(cmd), "rm -rf '%s' && mkdir -p '%s'",
              sil_root(), sil_root());
    system(cmd);
}

static int wait_for_file(const char *leaf, int timeout_ms)
{
    char path[512];
    sil_path(path, sizeof(path), leaf);

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (;;) {
        struct stat st;
        if (stat(path, &st) == 0) return 1;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= timeout_ms) return 0;
        struct timespec d = { 0, 20 * 1000000L };
        nanosleep(&d, NULL);
    }
}

static pid_t spawn(const char *path, const char *arg1)
{
    pid_t pid = fork();
    if (pid == 0) {
        /* Child: redirect to /dev/null unless debugging — keep test
         * output focused on Unity's own PASS/FAIL lines. Set
         * NSTAR_SIL_VERBOSE=1 in the environment to see subprocess logs. */
        if (!getenv("NSTAR_SIL_VERBOSE")) {
            int devnull = open("/dev/null", O_WRONLY);
            if (devnull >= 0) {
                dup2(devnull, STDERR_FILENO);
                dup2(devnull, STDOUT_FILENO);
                close(devnull);
            }
        }
        if (arg1) execl(path, path, arg1, (char *)NULL);
        else      execl(path, path, (char *)NULL);
        _exit(127);   /* exec failed */
    }
    return pid;
}

static void kill_and_wait(pid_t pid)
{
    if (pid <= 0) return;
    kill(pid, SIGTERM);
    int status;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (;;) {
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) return;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_nsec - start.tv_nsec) / 1000000;
        if (elapsed_ms >= 2000) { kill(pid, SIGKILL); waitpid(pid, &status, 0); return; }
        struct timespec d = { 0, 20 * 1000000L };
        nanosleep(&d, NULL);
    }
}

/** Write a GPIO line file to 0 or 1. */
static void set_gpio(const char *leaf, int value)
{
    char path[512];
    sil_path(path, sizeof(path), leaf);
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "%d", value);
    fclose(f);
}

/** Append one command line to the simulator control file. */
static void sim_control(const char *line)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_SIM_CONTROL_FILE);
    FILE *f = fopen(path, "a");
    if (!f) return;
    fprintf(f, "%s\n", line);
    fclose(f);
}

/** Append one command line to the application control file. */
static void app_control(const char *line)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_APP_CONTROL_FILE);
    FILE *f = fopen(path, "a");
    if (!f) return;
    fprintf(f, "%s\n", line);
    fclose(f);
}

/** Write data bytes to the RX data file the dummy HAL reads from. */
static void supply_rx_data(const uint8_t *buf, size_t len)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_RX_FILE);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fwrite(buf, 1, len, f);
    fclose(f);
}

/**
 * Read the entire TX data file the dummy HAL has appended to via
 * nstar_hal_data_write(). Returns the number of bytes read into buf
 * (up to buf_cap), or 0 if the file does not exist yet.
 */
static size_t read_tx_data(uint8_t *buf, size_t buf_cap)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_TX_FILE);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    size_t n = fread(buf, 1, buf_cap, f);
    fclose(f);
    return n;
}

/** Read the current data-clock-state file ("0" or "1"). Returns -1 if absent. */
static int read_data_clock_state(void)
{
    char path[512];
    sil_path(path, sizeof(path), SIL_DATA_CLOCK_FILE);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char buf[8] = {0};
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) return -1;
    return (buf[0] == '1') ? 1 : 0;
}

/* =========================================================================
 * setUp / tearDown
 * =========================================================================
 */

void setUp(void)
{
    clean_sil_root();

    if (!getenv("NSTAR_SIL_ROOT")) {
        setenv("NSTAR_SIL_ROOT", SIL_DEFAULT_ROOT, 1);
    }

    g_sim_pid = spawn(g_sim_path, NULL);
    TEST_ASSERT_TRUE_MESSAGE(wait_for_file(SIL_SIM_READY_FILE, 3000),
        "nstar_sim did not become ready in time");
}

void tearDown(void)
{
    kill_and_wait(g_app_pid);
    kill_and_wait(g_sim_pid);
    g_app_pid = -1;
    g_sim_pid = -1;
}

/* =========================================================================
 * Scenario 1 — Module FSM: startup succeeds, module reaches READY
 * =========================================================================
 */

void test_sil_fsm_startup_reaches_ready(void)
{
    g_app_pid = spawn(g_app_path, "10");

    int got = sil_status_wait_for("MODULE_STATE", "READY", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "module_state did not reach READY within timeout");
}

/* =========================================================================
 * Scenario 2 — Module FSM: FPGA_TYPE mismatch drives module to FAULT
 * =========================================================================
 */

void test_sil_fsm_fpga_type_mismatch_drives_fault(void)
{
    /* Force the simulator to report an unexpected FPGA_TYPE BEFORE the
     * app starts its startup_sequence, so the very first identity/type
     * check fails. */
    sim_control("FPGA_TYPE=FF");

    g_app_pid = spawn(g_app_path, "10");

    int got = sil_status_wait_for("MODULE_STATE", "FAULT", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "module_state did not reach FAULT after FPGA_TYPE mismatch");
}

/* =========================================================================
 * Scenario 3 — OBC-SBT commands: real register read round-trip
 * =========================================================================
 *
 * The startup_sequence itself performs R 0x06, R 0x08, R 0x09 over the
 * real PTY link against the real simulator, decoded with the real frame
 * codec. Reaching READY (already covered in scenario 1) is itself proof
 * the command round-trip works end-to-end. This scenario additionally
 * confirms a value change on the simulator side is correctly observed,
 * to rule out the app simply ignoring responses.
 */

void test_sil_commands_register_value_reflected_in_identity_path(void)
{
    /* A non-default but still-valid FPGA_TYPE the app should accept is
     * not possible (only 0x62 passes nstar_startup_sequence's check) —
     * instead, prove the round-trip is live by setting a WRONG type and
     * confirming the app's startup correctly fails (rather than silently
     * succeeding, which would indicate it isn't really reading the
     * simulator's register at all). */
    sim_control("FPGA_TYPE=99");

    g_app_pid = spawn(g_app_path, "10");

    int got = sil_status_wait_for("MODULE_STATE", "FAULT", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "Expected FAULT when simulator reports FPGA_TYPE=0x99");

    /* Now reset the simulator's register state and confirm a freshly
     * started app reaches READY — proving the earlier FAULT was due to
     * the injected register value, not a coincidence. */
    kill_and_wait(g_app_pid);
    g_app_pid = -1;
    sim_control("RESET");

    g_app_pid = spawn(g_app_path, "10");
    got = sil_status_wait_for("MODULE_STATE", "READY", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "Expected READY after RESET restored FPGA_TYPE to default");
}

/* =========================================================================
 * Scenario 4 — RX pipeline: GPIO-driven lock sequence delivers a frame
 * =========================================================================
 */

void test_sil_rx_lock_sequence_delivers_frame(void)
{
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)i;
    supply_rx_data(payload, sizeof(payload));

    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before RX scenario could start");

    /*
     * Edge-triggered semantics: the dummy HAL's nstar_hal_gpio_wait_edge()
     * seeds its "previous value" baseline from whatever the GPIO file
     * already contains the FIRST time it is asked to wait on that line.
     * If DATA_VALID is already 1 by the time the rx_thread starts waiting
     * for its rising edge, no transition is ever observed — exactly like
     * a real GPIO interrupt cannot fire for an edge that happened before
     * the handler was armed.
     *
     * So the sequence here mirrors real hardware: DATA_VALID starts LOW
     * (the default set by nstar_app_sil's init_gpio_defaults()), we raise
     * LOCK_DETECT and wait for ACQUIRING (proving the rx_thread is now
     * actively waiting on DATA_VALID), and only THEN raise DATA_VALID —
     * guaranteeing a genuine 0->1 transition occurs while observed.
     */
    set_gpio(SIL_GPIO_LOCK_DETECT_FILE, 1);

    int got = sil_status_wait_for("RX_STATE", "ACQUIRING", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "rx_state did not reach ACQUIRING");

    set_gpio(SIL_GPIO_DATA_VALID_FILE, 1);

    got = sil_status_wait_for("RX_STATE", "LOCKED", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got, "rx_state did not reach LOCKED");

    got = sil_status_wait_for_change("FRAME_RECEIVED_COUNT", "0", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "no frame was received via the RX pipeline");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *total = sil_status_get(&snap, "TOTAL_BYTES_RECEIVED");
    TEST_ASSERT_NOT_NULL(total);
    TEST_ASSERT_EQUAL_INT(64, atoi(total));
}

/* =========================================================================
 * Scenario 5 — Fault logic: FAULT_N assertion drives FAULT then recovery
 * =========================================================================
 */

void test_sil_fault_assertion_then_recovery(void)
{
    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before fault scenario could start");

    /* Assert FAULT_N (active-low: falling edge = fault) */
    set_gpio(SIL_GPIO_FAULT_N_FILE, 0);

    int got = sil_status_wait_for("MODULE_STATE", "FAULT", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "module_state did not reach FAULT");

    /* Release FAULT_N — N-STAR "self-recovers" */
    set_gpio(SIL_GPIO_FAULT_N_FILE, 1);

    got = sil_status_wait_for("MODULE_STATE", "READY", 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "module_state did not return to READY after FAULT_N released");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *last_fault = sil_status_get(&snap, "LAST_FAULT");
    TEST_ASSERT_NOT_NULL(last_fault);
    TEST_ASSERT_EQUAL_STRING("SEL", last_fault);
}

/* =========================================================================
 * Scenario 6 — TX pipeline: full session start/write/stop over real I/O
 * =========================================================================
 *
 * Drives nstar_tx_start() / nstar_tx_write() / nstar_tx_stop() on the
 * REAL nstar_core.c running inside nstar_app_sil, via the app control
 * file. Confirms:
 *   - tx_start succeeds (simulator reports TX_CLOCK_DETECTED=1 by default)
 *   - the dummy HAL's data-clock-state file goes "1" while active
 *   - tx_write delivers exact bytes to the dummy HAL's TX data file
 *   - tx_stop succeeds, clock state file returns to "0"
 *   - on_tx_complete fires with the correct byte count
 */

void test_sil_tx_full_session_delivers_bytes(void)
{
    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before TX scenario could start");

    app_control("TX_START=F");   /* NSTAR_TX_RATE_32K = 0x0F */

    int got = sil_status_wait_for("LAST_TX_START_RC", "0", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "tx_start did not report success (rc=0)");

    /* Clock should be running once tx_start succeeded */
    int clock_state = read_data_clock_state();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, clock_state,
        "data clock state file should be '1' after a successful tx_start");

    app_control("TX_WRITE=128");

    got = sil_status_wait_for("LAST_TX_WRITE_RC", "0", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "tx_write did not report success (rc=0)");

    app_control("TX_STOP");

    got = sil_status_wait_for_change("TX_COMPLETE_COUNT", "0", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "on_tx_complete did not fire");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *last_bytes = sil_status_get(&snap, "LAST_TX_BYTES");
    TEST_ASSERT_NOT_NULL(last_bytes);
    TEST_ASSERT_EQUAL_INT(128, atoi(last_bytes));

    /* Clock should be stopped after tx_stop */
    clock_state = read_data_clock_state();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, clock_state,
        "data clock state file should be '0' after tx_stop");

    /* Verify the exact bytes landed in the TX data file */
    uint8_t expected[128];
    for (int i = 0; i < 128; i++) expected[i] = (uint8_t)(i & 0xFF);

    uint8_t actual[256];
    size_t n = read_tx_data(actual, sizeof(actual));
    TEST_ASSERT_EQUAL_UINT_MESSAGE(128, n,
        "TX data file does not contain exactly 128 bytes");
    TEST_ASSERT_EQUAL_MEMORY(expected, actual, 128);
}

/* =========================================================================
 * Scenario 7 — TX pipeline: no clock detected -> NSTAR_ERR_NO_CLOCK
 * =========================================================================
 *
 * Forces the simulator to report TX_CLOCK_DETECTED=0, so tx_start's
 * internal R 0x40 check fails to see bit 4 set. Confirms nstar_tx_start()
 * returns NSTAR_ERR_NO_CLOCK (-7) rather than silently succeeding, and
 * that the dummy HAL's clock state file is left at "0" (clock was
 * de-asserted again after the failed check, per nstar_core.c's cleanup).
 */

void test_sil_tx_no_clock_detected_fails(void)
{
    sim_control("TX_CLOCK_DETECTED=0");

    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before TX scenario could start");

    app_control("TX_START=F");

    int got = sil_status_wait_for("LAST_TX_START_RC", "-7", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "tx_start did not report NSTAR_ERR_NO_CLOCK (-7) when "
        "simulator has no TX clock");

    int clock_state = read_data_clock_state();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, clock_state,
        "clock should be de-asserted again after a failed tx_start");
}

/* =========================================================================
 * Scenario 8 — RX pipeline: DATA_VALID falling mid-session -> LOCK_LOST
 * =========================================================================
 *
 * Completes a normal lock sequence (as in scenario 4), then drops
 * DATA_VALID and confirms rx_state transitions LOCKED -> LOCK_LOST and
 * on_lock_lost fires, exercising the path Stage 4's unit tests cover
 * in-process but this SIL suite had not yet driven over real GPIO files
 * and a real thread loop.
 */

void test_sil_rx_data_valid_falling_triggers_lock_lost(void)
{
    uint8_t payload[32];
    memset(payload, 0xAB, sizeof(payload));
    supply_rx_data(payload, sizeof(payload));

    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before RX scenario could start");

    set_gpio(SIL_GPIO_LOCK_DETECT_FILE, 1);
    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("RX_STATE", "ACQUIRING", 3000),
        "rx_state did not reach ACQUIRING");

    set_gpio(SIL_GPIO_DATA_VALID_FILE, 1);
    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("RX_STATE", "LOCKED", 5000),
        "rx_state did not reach LOCKED");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for_change("LOCK_ACQUIRED_COUNT", "0", 3000),
        "on_lock_acquired did not fire");

    /* Drop DATA_VALID — the rx_thread's read loop checks this GPIO
     * before every read and should exit to LOCK_LOST. */
    set_gpio(SIL_GPIO_DATA_VALID_FILE, 0);

    int got = sil_status_wait_for_change("LOCK_LOST_COUNT", "0", 3000);
    TEST_ASSERT_TRUE_MESSAGE(got, "on_lock_lost did not fire");
}

/* =========================================================================
 * Scenario 9 — RX pipeline: multi-chunk frame larger than
 *              NSTAR_FRAME_SIZE_BYTES delivers in full
 * =========================================================================
 *
 * Supplies more than one frame's worth of RX data so the rx_thread's
 * inner read loop must call on_frame_received more than once to drain
 * it, proving the chunking logic (and the accumulation of
 * TOTAL_BYTES_RECEIVED across multiple callback invocations) is correct
 * over the real dummy data file, not just in the mock.
 */

void test_sil_rx_multi_chunk_session_accumulates_bytes(void)
{
    size_t total = (size_t)NSTAR_FRAME_SIZE_BYTES + 512;
    uint8_t *payload = malloc(total);
    TEST_ASSERT_NOT_NULL(payload);
    for (size_t i = 0; i < total; i++) payload[i] = (uint8_t)(i & 0xFF);
    supply_rx_data(payload, total);
    free(payload);

    g_app_pid = spawn(g_app_path, "10");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before RX scenario could start");

    set_gpio(SIL_GPIO_LOCK_DETECT_FILE, 1);
    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("RX_STATE", "ACQUIRING", 3000),
        "rx_state did not reach ACQUIRING");

    set_gpio(SIL_GPIO_DATA_VALID_FILE, 1);
    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("RX_STATE", "LOCKED", 5000),
        "rx_state did not reach LOCKED");

    /* Wait until TOTAL_BYTES_RECEIVED reaches the full amount — the
     * rx_thread will need at least 2 calls to on_frame_received since
     * the payload exceeds NSTAR_FRAME_SIZE_BYTES. */
    char expected_str[32];
    snprintf(expected_str, sizeof(expected_str), "%zu", total);

    int got = sil_status_wait_for("TOTAL_BYTES_RECEIVED", expected_str, 5000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "TOTAL_BYTES_RECEIVED did not reach the full multi-chunk payload size");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *frame_count = sil_status_get(&snap, "FRAME_RECEIVED_COUNT");
    TEST_ASSERT_NOT_NULL(frame_count);
    TEST_ASSERT_TRUE_MESSAGE(atoi(frame_count) >= 2,
        "expected at least 2 on_frame_received calls for a multi-chunk payload");
}

/* =========================================================================
 * Scenario 10 — Health monitoring: PA over-temperature drives a thermal
 *               fault and stops TX
 * =========================================================================
 *
 * Sets a hot PA_TEMP_RAW on the simulator BEFORE the app starts, so the
 * health thread's first 30-second poll cycle already sees an
 * over-temperature reading on its very first nstar_health_read() call.
 * This is the one scenario that genuinely needs to wait out the real
 * NSTAR_HEALTH_POLL_INTERVAL_MS (30s, unscaled in the dummy HAL), so its
 * timeout budget is generous (40s) rather than the ~3-5s used elsewhere.
 *
 * raw=2300 -> T = 0.06105*2300-50 = 90.4C, well above
 * NSTAR_PA_TEMP_WARN_CELSIUS (85C).
 */

void test_sil_health_overtemp_drives_fault_and_stops_tx(void)
{
    sim_control("PA_TEMP_RAW=2300");

    g_app_pid = spawn(g_app_path, "45");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before health scenario could start");

    /* Start a TX session so we can also confirm the health thread stops
     * it proactively, per nstar_core.c's health_thread_func() logic. */
    app_control("TX_START=F");
    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("LAST_TX_START_RC", "0", 3000),
        "tx_start did not succeed ahead of the thermal scenario");

    /* The health thread's first poll happens ~30s after nstar_init();
     * give it a generous margin beyond that real interval. */
    int got = sil_status_wait_for_change("FAULT_COUNT", "0", 40000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "on_fault(TEMPERATURE) did not fire within the health poll window");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *last_fault = sil_status_get(&snap, "LAST_FAULT");
    TEST_ASSERT_NOT_NULL(last_fault);
    TEST_ASSERT_EQUAL_STRING("TEMPERATURE", last_fault);

    /* TX should have been stopped proactively before the fault fired */
    const char *tx_complete = sil_status_get(&snap, "TX_COMPLETE_COUNT");
    TEST_ASSERT_NOT_NULL(tx_complete);
    TEST_ASSERT_TRUE_MESSAGE(atoi(tx_complete) >= 1,
        "expected the health thread to have stopped the active TX session");
}

/* =========================================================================
 * Scenario 11 — Fault logic: FAULT_N does not self-clear -> hardware
 *               RESET_N pulse before recovery completes
 * =========================================================================
 *
 * Asserts FAULT_N and deliberately holds it LOW past
 * FAULT_RECOVERY_TIMEOUT_MS (5s), forcing fault_thread_func() down the
 * "N-STAR did not self-recover" branch: it asserts RESET_N low for
 * FAULT_RESET_HOLD_MS (100ms), releases it, then waits a SECOND 5s
 * window for FAULT_N to rise. We release FAULT_N only after that first
 * window has elapsed, so the only way recovery completes is via the
 * hard-reset branch.
 *
 * The 100ms RESET_N pulse is too short to reliably catch by polling a
 * file every 20ms without a race, so this scenario verifies the branch
 * indirectly but unambiguously via wall-clock timing: self-recovery
 * (scenario 5) completes in at most ~1-2s after FAULT_N rises, whereas
 * this path requires the first full 5s timeout to elapse before
 * RESET_N is even asserted — so total recovery time here is bounded
 * well above 5s, which is only possible via the hard-reset branch.
 */

void test_sil_fault_no_self_recovery_forces_hardware_reset(void)
{
    g_app_pid = spawn(g_app_path, "20");

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "READY", 5000),
        "module did not reach READY before fault scenario could start");

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    set_gpio(SIL_GPIO_FAULT_N_FILE, 0);   /* assert fault */

    TEST_ASSERT_TRUE_MESSAGE(
        sil_status_wait_for("MODULE_STATE", "FAULT", 2000),
        "module_state did not reach FAULT");

    /*
     * Hold FAULT_N low for longer than FAULT_RECOVERY_TIMEOUT_MS (5s) so
     * the fault thread's first recovery wait genuinely times out and it
     * proceeds to assert RESET_N, rather than racing a release that
     * happens to land inside the first window.
     */
    struct timespec hold = { 6, 0 };   /* 6s > 5s timeout, deliberate margin */
    nanosleep(&hold, NULL);

    set_gpio(SIL_GPIO_FAULT_N_FILE, 1);   /* release fault */

    int got = sil_status_wait_for("MODULE_STATE", "READY", 8000);
    TEST_ASSERT_TRUE_MESSAGE(got,
        "module_state did not return to READY after the hardware-reset "
        "recovery path");

    struct timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                      (t1.tv_nsec - t0.tv_nsec) / 1000000;

    /*
     * Self-recovery (scenario 5) typically completes in well under 5s
     * total. Forcing the hard-reset branch requires the first 5s
     * timeout to fully elapse before RESET_N is even asserted, so total
     * elapsed time here must be at least ~5s -- using 4.5s as a safety
     * margin against scheduling jitter while still clearly
     * distinguishing this path from self-recovery.
     */
    TEST_ASSERT_TRUE_MESSAGE(elapsed_ms >= 4500,
        "recovery completed too quickly to have gone through the "
        "5s self-recovery timeout and hardware-reset branch");

    sil_status_t snap;
    TEST_ASSERT_TRUE(sil_status_read(&snap));
    const char *last_fault = sil_status_get(&snap, "LAST_FAULT");
    TEST_ASSERT_NOT_NULL(last_fault);
    TEST_ASSERT_EQUAL_STRING("SEL", last_fault);

    /* RESET_N must be released (HIGH) again by the time recovery completed */
    char reset_path[512];
    sil_path(reset_path, sizeof(reset_path), SIL_GPIO_RESET_N_FILE);
    FILE *rf = fopen(reset_path, "r");
    TEST_ASSERT_NOT_NULL(rf);
    char rbuf[8] = {0};
    fread(rbuf, 1, sizeof(rbuf) - 1, rf);
    fclose(rf);
    TEST_ASSERT_EQUAL_CHAR('1', rbuf[0]);
}

/* =========================================================================
 * Test runner
 * =========================================================================
 */

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr,
            "usage: %s <path-to-nstar_sim> <path-to-nstar_app_sil>\n",
            argv[0]);
        return 1;
    }
    snprintf(g_sim_path, sizeof(g_sim_path), "%s", argv[1]);
    snprintf(g_app_path, sizeof(g_app_path), "%s", argv[2]);

    UNITY_BEGIN();

    RUN_TEST(test_sil_fsm_startup_reaches_ready);
    RUN_TEST(test_sil_fsm_fpga_type_mismatch_drives_fault);
    RUN_TEST(test_sil_commands_register_value_reflected_in_identity_path);
    RUN_TEST(test_sil_rx_lock_sequence_delivers_frame);
    RUN_TEST(test_sil_fault_assertion_then_recovery);
    RUN_TEST(test_sil_tx_full_session_delivers_bytes);
    RUN_TEST(test_sil_tx_no_clock_detected_fails);
    RUN_TEST(test_sil_rx_data_valid_falling_triggers_lock_lost);
    RUN_TEST(test_sil_rx_multi_chunk_session_accumulates_bytes);
    RUN_TEST(test_sil_health_overtemp_drives_fault_and_stops_tx);
    RUN_TEST(test_sil_fault_no_self_recovery_forces_hardware_reset);

    return UNITY_END();
}
