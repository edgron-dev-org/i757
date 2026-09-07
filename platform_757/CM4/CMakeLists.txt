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
set(STM32_MCU_FLAGS  "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard" )

#Linker options
set(STM32_LINKER_SCRIPT stm32h757xx_flash_CM4.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME platform_757_CM4)
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
    HAL_IWDG_MODULE_ENABLED   # P0: IWDG2 owned by App code
    METAL_INTERNAL
    METAL_MAX_DEVICE_REGIONS=2
    NO_ATOMIC_64_SUPPORT
    RPMSG_BUFFER_SIZE=512
    VIRTIO_SLAVE_ONLY         # CM4=remote (guide §3)
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/OpenAMP/open-amp/lib/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/OpenAMP/libmetal/lib/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common/openamp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common         # app_pimage.h: dual-core process-image shared layout
    ${CMAKE_CURRENT_SOURCE_DIR}/../../uartdrv    # P2②: backplane UART8 low-level driver
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modbus     # P2②: Modbus core (pure C)
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/app_p0_cm4.c
    App/app_rpc_cm4.c
    App/app_cm4_stubs.c
    App/app_mbport_cm4.c                          # P2②: backplane master stack (UART8+DE=PD11)
    App/app_mbfront_cm4.c                         # front panel 6×485 configurable Modbus ports + board-level slave map (2026-07-18)
    App/app_pscan_cm4.c                           # process-image scanner (data plane, SRAM4 image, 2026-07-19)
    App/app_diag_cm4.c                            # op0D self-report: tasks / stack headroom / heap
    App/app_platform_cm4.c                        # CM4-side application API (mirrors CM7 app_platform.h)
    App/app_user_cm4.c                            # YOUR application on the realtime core (app_user_cm4_init)
    App/app_bus_cm4.c                             # one thread per Modbus bus (fault isolation, 2026-07-19)
    App/app_hsdi_cm4.c                            # onboard 8ch HSDI: DI / counter / encoder (2026-07-19)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../uartdrv/uart_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modbus/modbus_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_iwdg.c
)

# ---- OpenAMP/RPMsg CM4 side (P2① 2026-07-17, guide=Common/openamp/Integration_Guide.md) ----
set(OAMP ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/OpenAMP)
set(OAMP_GLUE ${CMAKE_CURRENT_SOURCE_DIR}/../Common/openamp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${OAMP}/libmetal/lib/device.c ${OAMP}/libmetal/lib/init.c ${OAMP}/libmetal/lib/io.c
    ${OAMP}/libmetal/lib/log.c ${OAMP}/libmetal/lib/shmem.c
    ${OAMP}/libmetal/lib/system/generic/condition.c ${OAMP}/libmetal/lib/system/generic/irq.c
    ${OAMP}/libmetal/lib/system/generic/time.c ${OAMP}/libmetal/lib/system/generic/generic_device.c
    ${OAMP}/libmetal/lib/system/generic/generic_init.c ${OAMP}/libmetal/lib/system/generic/generic_io.c
    ${OAMP}/libmetal/lib/system/generic/cortexm/sys.c
    ${OAMP}/open-amp/lib/remoteproc/remoteproc_virtio.c
    ${OAMP}/open-amp/lib/rpmsg/rpmsg.c ${OAMP}/open-amp/lib/rpmsg/rpmsg_virtio.c
    ${OAMP}/open-amp/lib/virtio/virtio.c ${OAMP}/open-amp/lib/virtio/virtqueue.c
    ${OAMP_GLUE}/openamp.c ${OAMP_GLUE}/mbox_hsem.c ${OAMP_GLUE}/rsc_table.c)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()









