cmake_minimum_required(VERSION 3.16)
project(nstar_interface C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(NSTAR_INC     ${CMAKE_SOURCE_DIR}/inc)
set(NSTAR_SRC     ${CMAKE_SOURCE_DIR}/src)   # absolute — fixes relative-path issue

# ---------------------------------------------------------------------------
# nstar_frame — CRC ENABLED
# ---------------------------------------------------------------------------
add_library(nstar_frame_crc_on STATIC src/nstar_frame.c)
target_include_directories(nstar_frame_crc_on PUBLIC ${NSTAR_INC})

# ---------------------------------------------------------------------------
# nstar_frame — CRC DISABLED
# ---------------------------------------------------------------------------
add_library(nstar_frame_crc_off STATIC src/nstar_frame.c)
target_include_directories(nstar_frame_crc_off PUBLIC ${NSTAR_INC})
target_compile_definitions(nstar_frame_crc_off PUBLIC NSTAR_CRC_DISABLED)

# ---------------------------------------------------------------------------
# nstar_core (Stage 2: synchronous command interface)
# ---------------------------------------------------------------------------
add_library(nstar_core STATIC src/nstar_core.c)
target_include_directories(nstar_core PUBLIC ${NSTAR_INC})
target_link_libraries(nstar_core PUBLIC nstar_frame_crc_on pthread m)

# ---------------------------------------------------------------------------
# HAL: mock  (absolute path for src/ fixes the nstar_hal_mock.h include)
# ---------------------------------------------------------------------------
add_library(nstar_hal_mock STATIC src/nstar_hal_mock.c)
target_include_directories(nstar_hal_mock PUBLIC ${NSTAR_INC} ${NSTAR_SRC})

# ---------------------------------------------------------------------------
# HAL: real Linux (Stage 6 stubs)
# ---------------------------------------------------------------------------
add_library(nstar_hal_linux STATIC hal/nstar_hal_linux.c)
target_include_directories(nstar_hal_linux PUBLIC ${NSTAR_INC})

# ---------------------------------------------------------------------------
# Main binary
# ---------------------------------------------------------------------------
add_executable(nstar_app main/nstar_app.c)
target_include_directories(nstar_app PRIVATE ${NSTAR_INC})
target_link_libraries(nstar_app PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_linux pthread)

# ---------------------------------------------------------------------------
# Unity test framework
# ---------------------------------------------------------------------------
add_library(unity STATIC tests/unity/unity.c)
target_include_directories(unity PUBLIC tests/unity)

# ---------------------------------------------------------------------------
# Test: Stage 1 — frame codec CRC ENABLED
# ---------------------------------------------------------------------------
add_executable(test_frame_crc_on tests/test_frame.c)
target_include_directories(test_frame_crc_on PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_frame_crc_on PRIVATE
    nstar_frame_crc_on nstar_hal_mock unity)

# ---------------------------------------------------------------------------
# Test: Stage 1 — frame codec CRC DISABLED
# ---------------------------------------------------------------------------
add_executable(test_frame_crc_off tests/test_frame.c)
target_include_directories(test_frame_crc_off PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_frame_crc_off PRIVATE
    nstar_frame_crc_off nstar_hal_mock unity)

# ---------------------------------------------------------------------------
# Test: Stage 2 — command interface
# ---------------------------------------------------------------------------
add_executable(test_core tests/test_core.c)
target_include_directories(test_core PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_core PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_mock unity pthread)

# ---------------------------------------------------------------------------
# Test: Stage 3 — TX pipeline
# ---------------------------------------------------------------------------
add_executable(test_tx tests/test_tx.c)
target_include_directories(test_tx PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_tx PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_mock unity pthread m)

# ---------------------------------------------------------------------------
# Test: Stage 4 — RX pipeline
# ---------------------------------------------------------------------------
add_executable(test_rx tests/test_rx.c)
target_include_directories(test_rx PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_rx PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_mock unity pthread m)

# ---------------------------------------------------------------------------
# Test: Stage 5 — Health and fault handling
# ---------------------------------------------------------------------------
add_executable(test_health_fault tests/test_health_fault.c)
target_include_directories(test_health_fault PRIVATE ${NSTAR_INC} ${NSTAR_SRC})
target_link_libraries(test_health_fault PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_mock unity pthread m)

# ---------------------------------------------------------------------------
# SIL (Software-In-the-Loop) testing
# ---------------------------------------------------------------------------
# Three additional binaries, isolated from the Stage 1-5 unit test build:
#   nstar_sim       — standalone N-STAR protocol simulator (separate process)
#   nstar_app_sil   — application under test, real nstar_core.c + dummy HAL
#   test_sil        — Unity-based black-box test suite, orchestrates both
#
# hal/nstar_hal_dummy.c provides real PTY UART I/O, file-polled GPIO, and
# file-based data interface I/O — swapped in place of nstar_hal_linux.c's
# (currently stubbed) real hardware bindings.
# ---------------------------------------------------------------------------

set(NSTAR_SIL ${CMAKE_SOURCE_DIR}/sil)

add_library(nstar_hal_dummy STATIC hal/nstar_hal_dummy.c)
target_include_directories(nstar_hal_dummy PUBLIC ${NSTAR_INC} ${NSTAR_SIL})

add_executable(nstar_sim sil/nstar_sim.c)
target_include_directories(nstar_sim PRIVATE ${NSTAR_INC} ${NSTAR_SIL})
target_link_libraries(nstar_sim PRIVATE nstar_frame_crc_on util)

add_executable(nstar_app_sil sil/nstar_app_sil.c)
target_include_directories(nstar_app_sil PRIVATE ${NSTAR_INC} ${NSTAR_SIL})
target_link_libraries(nstar_app_sil PRIVATE
    nstar_core nstar_frame_crc_on nstar_hal_dummy pthread m)

add_executable(test_sil sil/test_sil.c)
target_include_directories(test_sil PRIVATE ${NSTAR_INC} ${NSTAR_SIL} ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(test_sil PRIVATE unity)

# ---------------------------------------------------------------------------
# CTest
# ---------------------------------------------------------------------------
enable_testing()
add_test(NAME stage1_frame_CRC_ON  COMMAND test_frame_crc_on)
add_test(NAME stage1_frame_CRC_OFF COMMAND test_frame_crc_off)
add_test(NAME stage2_command       COMMAND test_core)
add_test(NAME stage3_tx            COMMAND test_tx)
add_test(NAME stage4_rx            COMMAND test_rx)
add_test(NAME stage5_health_fault  COMMAND test_health_fault)
add_test(NAME sil_full_suite       COMMAND test_sil
         $<TARGET_FILE:nstar_sim> $<TARGET_FILE:nstar_app_sil>)
