cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

#Linker options
set(STM32_LINKER_SCRIPT stm32h757xx_flash_CM7.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME platform_757_CM7)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")


# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    HAL_IWDG_MODULE_ENABLED   # IWDG owned by App code (modular HAL mechanism, does not touch hal_conf.h)
    APP_SE_BACKEND_608B       # anti-clone SE backend: 2026-07-18 switched to real 608A (app_se_608b->atecc_sign)
    LFS_NO_MALLOC             # LittleFS fully static buffers, disable malloc path at compile time
    LFS_NO_DEBUG              # keep only WARN/ERROR prints (saves flash strings + CDC noise)
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
    App/littlefs              # LittleFS v2.9.3 imported as-is (SPI Flash file system)
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common         # app_pimage.h: dual-core process-image shared layout
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modbus     # P2②: Modbus core (pure C)
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/app_p0.c
    App/app_ota.c
    App/app_mqtt.c
    App/app_user.c                                # YOUR application entry point (app_user_init)
    App/examples/example_dosing.c                 # manual §4.4, compiled so the example can never rot (entry namespaced, not run)
    App/app_init.c
    App/app_time.c
    App/app_anticlone.c
    App/app_se_608b.c
    App/app_se_ecdsa_alt.c                        # MBEDTLS_ECDSA_SIGN_ALT: TLS handshake signature forwarded to 608A (tls-608a branch)
    App/app_p1_stubs.c
    App/app_rpc.c
    App/app_p5_test.c
    App/app_cli.c
    App/app_usb_fw.c
    App/app_cdc_console.c
    App/app_leds_beep.c
    App/app_qflash.c
    App/app_lfs.c
    App/app_identity.c                            # device identity partition (universal image; jig writes identity.der)
    App/app_mbport.c                              # P2②: M7-side Modbus client (delivers to CM4 via RPMsg)
    App/app_mbcfg.c                               # generic Modbus port config (littlefs+op8+CLI/cloud, 2026-07-18)
    App/app_netcfg.c                              # static-fallback IP runtime config (littlefs net.cfg, 2026-08-31)
    App/app_pimage_cm7.c                          # process-image I/O scanner API (app_io_*, SRAM4 image, 2026-07-19)
    App/app_diag.c                                # 'tasks' CLI: both cores' stacks + heap (2026-07-19)
    App/app_hsdi.c                                # onboard HSDI CLI view (channels owned by CM4, 2026-07-19)
    App/app_pwrfail_test.c                        # power-fail three-layer retention mechanism (API=app_pwrfail.h; predecessor=window measurement, 2026-07-18)
    App/app_sec_test.c                            # dual secure-element first test (608A wake/SE050 ENA, CLI sec)
    App/app_mbtcp.c                               # Modbus TCP shell (502 server op9 tunnel + client)
    App/app_mota.c                                # expansion module OTA (littlefs repo + backplane streaming flash)
    App/app_datalog.c                             # data recorder + event journal (SD/littlefs daily CSV, 2026-08-21)
    App/app_vent.c                                # greenhouse roof-vent temperature control (3-stage window, 2026-08-27)
    App/app_dose.c                                # nutrient/pH dosing pumps (3x peristaltic on relay coils 0..2, 2026-08-31)
    App/app_flowmon.c                             # per-gutter feed-flow monitor: blocked-line alarm + buzzer + cloud event (2026-09-04)
    App/app_aer.c                                 # reservoir aeration: air pump on relay coil 8 switched on water temperature (2026-09-08)
    App/app_ffunicode.c                           # FatFs LFN unicode hooks, ASCII-only shim (_USE_LFN=1)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modbus/modbus_core.c
    App/littlefs/lfs.c
    App/littlefs/lfs_util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.c  # manually copied from FW_H7_V1.13.0 (originally missing in tree, H503 precedent)
)

# TLS integration (full mbedTLS library -O2 + altcp glue + build-time certificate generation) -- ported from nucleo, P2/P3 blocks already trimmed
include(${CMAKE_CURRENT_SOURCE_DIR}/App/app_tls.cmake)

# version nameplate auto-refresh (nucleo precedent): ensures app_init.c recompiles every build -> __DATE__/__TIME__ reflect this build moment
set(VERSION_STAMP ${CMAKE_CURRENT_BINARY_DIR}/version_stamp.h)
add_custom_target(gen_version_stamp
    COMMAND ${CMAKE_COMMAND} -E touch ${VERSION_STAMP}
    BYPRODUCTS ${VERSION_STAMP}
    COMMENT "Refresh version nameplate timestamp"
)
add_dependencies(${CMAKE_PROJECT_NAME} gen_version_stamp)
set_source_files_properties(App/app_init.c PROPERTIES OBJECT_DEPENDS ${VERSION_STAMP})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# OpenOCD FreeRTOS thread awareness: uxTopUsedPriority is const and unreferenced, would be dropped by --gc-sections -> force-keep it (debug infrastructure)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--undefined=uxTopUsedPriority)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()









