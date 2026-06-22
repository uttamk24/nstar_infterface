# CMake generated Testfile for 
# Source directory: /home/dhruvaspace/NSTAR-TTC
# Build directory: /home/dhruvaspace/NSTAR-TTC/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(stage1_frame_CRC_ON "/home/dhruvaspace/NSTAR-TTC/build/test_frame_crc_on")
set_tests_properties(stage1_frame_CRC_ON PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;139;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(stage1_frame_CRC_OFF "/home/dhruvaspace/NSTAR-TTC/build/test_frame_crc_off")
set_tests_properties(stage1_frame_CRC_OFF PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;140;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(stage2_command "/home/dhruvaspace/NSTAR-TTC/build/test_core")
set_tests_properties(stage2_command PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;141;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(stage3_tx "/home/dhruvaspace/NSTAR-TTC/build/test_tx")
set_tests_properties(stage3_tx PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;142;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(stage4_rx "/home/dhruvaspace/NSTAR-TTC/build/test_rx")
set_tests_properties(stage4_rx PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;143;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(stage5_health_fault "/home/dhruvaspace/NSTAR-TTC/build/test_health_fault")
set_tests_properties(stage5_health_fault PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;144;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
add_test(sil_full_suite "/home/dhruvaspace/NSTAR-TTC/build/test_sil" "/home/dhruvaspace/NSTAR-TTC/build/nstar_sim" "/home/dhruvaspace/NSTAR-TTC/build/nstar_app_sil")
set_tests_properties(sil_full_suite PROPERTIES  _BACKTRACE_TRIPLES "/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;145;add_test;/home/dhruvaspace/NSTAR-TTC/CMakeLists.txt;0;")
