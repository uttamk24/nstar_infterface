# N-STAR TTC Transponder Interface

OBC software interface for the Syrlinks N-STAR S/S-Band Transceiver/Transponder.  
Implements UART command/telemetry, TX/RX data pipelines, health monitoring and SEL fault recovery.

---

## Repository layout

```
nstar_out/
├── inc/
│   └── ttc_nstar.h          Public API — types, constants, function prototypes
├── src/
│   ├── ttc_nstar.c          Core logic — FSM, threads, command path, TX/RX pipelines
│   ├── ttc_nstar_frame.c    UART frame codec — encode/decode, CRC16-XMODEM
│   ├── ttc_nstar_hal_linux.c  Stage 6 HAL stubs — fill in for real hardware
│   ├── ttc_nstar_hal_dummy.c  SIL HAL — PTY UART, file GPIO, file data interface
│   └── ttc_nstar_hal_mock.c/h Unit-test HAL — deterministic in-process mock
├── main/
│   └── ttc_nstar.c          Application entry point — pubSubMessageProcessor + main()
├── sil/
│   ├── nstar_sim.c          N-STAR protocol simulator (separate process, PTY UART)
│   ├── nstar_app_sil.c      SIL application wrapper (links ttc_nstar_core + dummy HAL)
│   ├── test_sil.c           Black-box SIL test suite (11 scenarios)
│   ├── sil_common.h         Shared SIL file paths and helpers
│   └── sil_status.h         SIL GPIO / data interface file names
├── tests/
│   ├── test_frame.c         Frame codec unit tests (CRC on/off)
│   ├── test_core.c          Register access and startup sequence tests
│   ├── test_tx.c            TX pipeline tests
│   ├── test_rx.c            RX pipeline tests
│   ├── test_health_fault.c  Health monitor and fault recovery tests
│   └── unity/               Unity test framework (embedded copy)
└── CMakeLists.txt
```

---

## Hardware

**Device:** Syrlinks N-STAR S/S-Band Transceiver/Transponder  
**Reference documents:** EICD v1.4 · User Manual v1.3 · IRD v1.5

| Interface        | Signal          | Connector      | Direction |
|------------------|-----------------|----------------|-----------|
| Control (UART)   | HK_RX / HK_TX  | PC104 H1 47/51 | RS-422    |
| TX data + clock  | DATA_TX / CLK_TX| PC104 H1 / J2  | RS-422 or CMOS |
| RX data + clock  | DATA_RX / CLK_RX| PC104 H1 / J2  | RS-422 or CMOS |
| Carrier lock     | LOCK_DETECT     | PC104 H1 pin 20| CMOS output |
| Data valid       | DATA_VALID      | PC104 H1 pin 22| CMOS output |
| Fault            | FAULT_N         | PC104 H1 pin 24| Open-collector |
| Reset            | RESET_N         | PC104 H1 pin 28| CMOS input  |

**FAULT_N polarity:** HIGH = fault active (open-collector released, external 100 kΩ pull-up raises line).  
Normal idle state is LOW. See User Manual §3.1.1.

---

## Building

**Requirements:** GCC, CMake ≥ 3.16, `libutil-dev` (for the SIL PTY simulator)

```bash
sudo apt-get install -y build-essential cmake libutil-dev

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

### CMake targets

| Target | Output | Description |
|--------|--------|-------------|
| `ttc_nstar_core` | static lib | Core logic — link into flight application |
| `ttc_nstar_frame_crc_on` | static lib | Frame codec with CRC enabled |
| `ttc_nstar_hal_linux` | static lib | Real Linux HAL stubs (Stage 6, fill in) |
| `ttc_nstar_hal_dummy` | static lib | SIL HAL (PTY + file I/O) |
| `ttc_nstar_hal_mock` | static lib | Unit-test mock HAL |
| `ttc_nstar_app` | executable | Flight application entry point |
| `nstar_sim` | executable | N-STAR protocol simulator for SIL |
| `nstar_app_sil` | executable | SIL application under test |
| `test_sil` | executable | SIL black-box test suite |
| `test_frame_crc_on/off` | executables | Frame codec unit tests |
| `test_core/tx/rx/health_fault` | executables | Module unit tests |

---

## Running tests

### Unit tests (no hardware required)

```bash
cd build
ctest --output-on-failure
# or run individually:
./test_frame_crc_on
./test_core
./test_tx
./test_rx
./test_health_fault
```

### SIL full suite (no hardware required)

The SIL harness spawns `nstar_sim` (a protocol simulator) and `nstar_app_sil` (the module
under test) as separate processes communicating over a PTY, with GPIO signals emulated via files.

```bash
cd build
./test_sil ./nstar_sim ./nstar_app_sil
# with verbose output:
NSTAR_SIL_VERBOSE=1 ./test_sil ./nstar_sim ./nstar_app_sil
# or via ctest:
ctest -R sil_full_suite --output-on-failure
```

Expected output: **7/7 tests passed** (unit stages 1–5 + SIL full suite), runtime ~117 s.

---

## Module state machine

```
UNINIT ──NSTAR_Init()──► INITIALISING ──NSTAR_StartupSequence()──► STARTING
                                                                        │
         ◄──────────────────── FAULT ◄── any startup step fails ───────┤
         │  faultThread re-runs startup                                 │
         │                                               all steps OK ──►  READY
         └────────────────────────────────────────────────────────────────────┘
                            FAULT_N rising edge detected
```

### Startup sequence steps (inside `NSTAR_StartupSequence`)

1. Sleep 3 000 ms — oscillator warm-up (cold start only)
2. V command — read and cache FPGA identity (version, build, serial)
3. R 0x06 — verify `FPGA_TYPE == 0x62` (N-STAR PCM/PM RX+TX)
4. W 0x10 = 0x02 — configure 2-pass on-board frequency sweep

---

## API reference

### Lifecycle

```c
// Allocate context, spawn background threads, state → INITIALISING
NSTAR_Result_t NSTAR_Init(const NSTAR_Config_t *config,
                           const NSTAR_Callbacks_t *cbs,
                           NSTAR_Ctx_t **ctx_out);

// Run 4-step startup, state → READY on success
NSTAR_Result_t NSTAR_StartupSequence(NSTAR_Ctx_t *ctx);

// Stop threads, free context
void NSTAR_Deinit(NSTAR_Ctx_t *ctx);
```

### Register access

```c
NSTAR_Result_t NSTAR_RegRead(NSTAR_Ctx_t *ctx, uint8_t addr, uint8_t *val);
NSTAR_Result_t NSTAR_RegWrite(NSTAR_Ctx_t *ctx, uint8_t addr, uint8_t val);
NSTAR_Result_t NSTAR_RegReadMulti(NSTAR_Ctx_t *ctx, uint8_t startAddr,
                                   size_t count, uint8_t *buf);
```

### Multi-byte latching registers

These registers only take effect when the **final (LSB/latch) address** is written.
Always use the helper functions — never call `NSTAR_RegWrite` on these addresses individually.

| Helper | Latch address | Register |
|--------|--------------|----------|
| `NSTAR_SetRXSensitivity(ctx, rawValue)` | 0x13 | RX wake-up threshold (23-bit) |
| `NSTAR_SetTXFilter(ctx, filterConfig)` | 0x43 | TX FIR filter config (9-bit) |
| `NSTAR_SetRXFrequency(ctx, freqInt, freqFrac)` | 0x27 | RX carrier frequency (CFF option) |
| `NSTAR_SetTXFrequency(ctx, freqSel, freqInt, freqFrac)` | 0x65 | TX carrier frequency (CFF option) |

### TX pipeline

```c
// Assert TX clock, wait 100 ms, verify clock detected, enable modulation
NSTAR_Result_t NSTAR_TXStart(NSTAR_Ctx_t *ctx, NSTAR_TXRateCode_t rateCode);

// Write data in chunks ≤ 2048 bytes; gap between calls must be < 1 ms
NSTAR_Result_t NSTAR_TXWrite(NSTAR_Ctx_t *ctx, const uint8_t *buf, size_t len);

// Set standby, stop clock, fire onTXComplete callback
NSTAR_Result_t NSTAR_TXStop(NSTAR_Ctx_t *ctx);
```

### RX pipeline

```c
// Set RX data rate register
NSTAR_Result_t NSTAR_RXConfigure(NSTAR_Ctx_t *ctx, NSTAR_RXRateCode_t rateCode);

// Read 14-register RX status snapshot (E command)
NSTAR_Result_t NSTAR_CMDReadAllRXStatus(NSTAR_Ctx_t *ctx, NSTAR_RXStatus_t *out);

// Read Eb/N0, RSSI, frequency shift (call only when rxState == LOCKED)
NSTAR_Result_t NSTAR_RXGetLinkQuality(NSTAR_Ctx_t *ctx, NSTAR_LinkQuality_t *out);
```

### Health and fault

```c
// Read PA temp, BB temp, FAULT_N GPIO; fires NSTAR_TXStop if PA > 85 °C
NSTAR_Result_t NSTAR_HealthRead(NSTAR_Ctx_t *ctx, NSTAR_Health_t *out);

// Soft reset — resets all N-STAR registers to factory defaults
NSTAR_Result_t NSTAR_CMDReset(NSTAR_Ctx_t *ctx);
```

### Callbacks (registered at `NSTAR_Init`)

```c
typedef struct {
    void (*onFrameReceived)(const uint8_t *buf, size_t len); // RX data ready
    void (*onTXComplete)(size_t bytesSent);                  // TX session done
    void (*onFault)(NSTAR_FaultSource_t source);             // SEL or thermal
    void (*onLockAcquired)(void);                            // LOCK_DETECT ↑
    void (*onLockLost)(void);                                // DATA_VALID ↓
} NSTAR_Callbacks_t;
```

> **Do not call any `NSTAR_*` functions from inside a callback.** Callbacks fire
> from background threads that hold internal locks.

---

## Background threads

Three threads are spawned by `NSTAR_Init` and run until `NSTAR_Deinit`:

| Thread | Trigger | Action |
|--------|---------|--------|
| `rxThreadFunc` | GPIO edge on LOCK_DETECT / DATA_VALID | Manages RX state machine: IDLE → ACQUIRING → LOCKED → LOCK_LOST |
| `faultThreadFunc` | GPIO edge on FAULT_N (RISING = fault) | Stops TX, waits up to 5 s for auto-recovery, asserts RESET_N if needed, re-runs startup |
| `healthThreadFunc` | 30 s timer | Reads PA/BB temperatures and FAULT_N; stops TX if PA > 85 °C |

---

## HAL (Stage 6 — real hardware)

Fill in `src/ttc_nstar_hal_linux.c` for the target OBC. Two open points remain:

1. **Data interface** (`nstarDataWrite`, `nstarDataRead`, `nstarDataClockStart/Stop`) — physical interface to be confirmed with Syrlinks (RS-422 SBDL or CMOS).
2. **GPIO library** — choose `libgpiod` (recommended, kernel ≥ 4.8) or sysfs.

UART parameters (38400 baud, 8E1, RS-422) are fixed at manufacture.

```c
// Minimum config to pass to NSTAR_Init:
NSTAR_Config_t cfg = {
    .uartFd         = open("/dev/ttyS1", O_RDWR | O_NOCTTY),
    .gpioLockDetect = <fd>,   // LOCK_DETECT  PC104 H1 pin 20
    .gpioDataValid  = <fd>,   // DATA_VALID   PC104 H1 pin 22
    .gpioFaultN     = <fd>,   // FAULT_N      PC104 H1 pin 24
    .gpioResetN     = <fd>,   // RESET_N      PC104 H1 pin 28
    .dataFd         = -1,     // TBD
};
```

---

## Key constraints

| Parameter | Value | Source |
|-----------|-------|--------|
| UART baud rate | 38 400, 8E1, RS-422 | IRD [N-STAR_IRD_0003] |
| Command timeout | 500 ms end-to-end | IRD [N-STAR_IRD_0017] |
| Command receive window | 40 ms (UART) | IRD [N-STAR_IRD_0018] |
| TX clock pre-stable wait | ≥ 100 ms before requesting modulation | User Manual §3.1 |
| TX clock gap max | 1 ms — longer gap causes auto-standby | User Manual §3.2.1 |
| TX chunk size | ≤ 2048 bytes per `NSTAR_TXWrite` call | — |
| Oscillator warm-up | ≥ 3 s after cold power-on | User Manual §3.1 |
| PA thermal guard (SW) | 85 °C — software stops TX | — |
| PA thermal guard (HW) | 90 °C — hardware stops TX automatically | User Manual §2.2 |
| SEL auto-recovery window | 3 s minimum (hardware guarantee) | User Manual §3.1.1 |
| RESET_N hold time | 100 ms | EICD §5.4 |
| FAULT_N idle level | LOW (no fault) | EICD §5.4, User Manual §3.1.1 |
| FAULT_N active level | HIGH (fault asserted) | EICD §5.4, User Manual §3.1.1 |

---

## UART frame format

```
< CMD_ID DATA_SIZE : DATA [: CRC] >
```

- All fields ASCII hex-encoded (1 raw byte = 2 ASCII chars)
- CRC: CRC16-XMODEM over `<CMD_ID DATA_SIZE : DATA`
- Example register read: `<R02:10:9EA5>` → read register 0x10, response value 0x9E with CRC 0xA5

| Command | ID | Data sent | Data returned |
|---------|----|-----------|---------------|
| Read register | `R` | 1-byte address | 1-byte value |
| Write register | `W` | 2-byte [addr, val] | ACK |
| Reset | `C` | 2-byte magic (0x5A5A) | ACK |
| Read all RX status | `E` | none | 19-byte register dump |
| Read FPGA identity | `V` | none | 6-byte identity |

---

## SEL fault recovery sequence

```
FAULT_N ────────────┐ HIGH (fault)          ┌── LOW (recovered)
                    ↓                        │
faultThread:  state→FAULT, TXStop()         │
              nstarGPIOWaitEdge(FAULT_N, FALLING, 5000 ms)
                    │                        │
                    │ timeout?               │ auto-recovered
                    ▼                        │
              nstarGPIOWrite(RESET_N, LOW)   │
              sleep 100 ms                  │
              nstarGPIOWrite(RESET_N, HIGH) ─┘
                    │
                    └──► onFault(NSTAR_FAULT_SEL)
                         NSTAR_StartupSequence()   ← restores all registers
                         state → READY
```

> N-STAR resets all registers to factory defaults after any reset (hardware or software).
> `NSTAR_StartupSequence` must always be called after recovery to restore configuration.
