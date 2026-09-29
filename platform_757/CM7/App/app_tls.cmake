# app_tls.cmake — TLS build integration (user file; included from user region of CM7/CMakeLists.txt)
# ① full mbedTLS 2.16.2 library (POSIX-dependent net_sockets/timing removed)
# ② LwIP altcp_tls glue (from local ST firmware pack FW_H7_V1.13.0, same version as in-tree LwIP 2.2.1)
# ③ app_certs.c generated at build time from keys/ca/ into build directory —— device private key does not enter source tree/git

set(MBEDTLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/mbedTLS)
file(GLOB MBEDTLS_SRC ${MBEDTLS_DIR}/library/*.c)
list(REMOVE_ITEM MBEDTLS_SRC
    ${MBEDTLS_DIR}/library/net_sockets.c
    ${MBEDTLS_DIR}/library/timing.c)

set(ALTCP_TLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/LwIP/src/apps/altcp_tls)

# certificate generation: PEM -> C string literal
# Two sources:
#   keys/   — OUTSIDE the repo (../../../keys, never committed): private material and the
#             operator's per-board certificates. Present on Edgron build machines only.
#   certs/  — INSIDE the repo (../../certs, committed, nothing secret): the Edgron root CA
#             certificate and the Edgron firmware-signing PUBLIC key. A plain checkout with no
#             keys/ builds a firmware that connects to the Edgron broker with the board's factory
#             identity (608A + identity partition) and accepts Edgron-signed releases.
# keys/ wins when both exist. Anything found in neither becomes an empty stub (the build still
# succeeds; that feature is inert). APP_ENABLE_CLOUD=0 (app_cfg.h) needs none of this.
set(KEYS_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../../keys/ca)
set(CERTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../certs)
if(NOT EXISTS ${KEYS_DIR}/ca.crt)
    if(EXISTS ${CERTS_DIR}/ca.crt)
        message(STATUS "keys/ not found -> using the public certificates in certs/ (Edgron broker CA + release signing key). Anti-clone provisioning material is absent (inert).")
    else()
        message(WARNING "neither keys/ nor certs/ found -> building with STUB certificates: the cloud stays offline. For a standalone controller set APP_ENABLE_CLOUD=0 in App/app_cfg.h.")
    endif()
endif()
function(pem_to_c INFILE OUTVAR)
    if(EXISTS ${INFILE})
        file(READ ${INFILE} _pem)
        string(REPLACE "\n" "\\n\"\n\"" _pem "${_pem}")
        set(${OUTVAR} "\"${_pem}\"" PARENT_SCOPE)
    else()
        set(${OUTVAR} "\"\"" PARENT_SCOPE)   # stub: empty PEM when the file is absent
    endif()
endfunction()
# first existing file wins: the private keys/ copy, else the committed public copy in certs/
function(pem_to_c_first OUTVAR PRIVATE_FILE PUBLIC_FILE)
    if(EXISTS ${PRIVATE_FILE})
        pem_to_c(${PRIVATE_FILE} _v)
    else()
        pem_to_c(${PUBLIC_FILE} _v)
    endif()
    set(${OUTVAR} "${_v}" PARENT_SCOPE)
endfunction()
pem_to_c_first(CA_PEM_C ${KEYS_DIR}/ca.crt ${CERTS_DIR}/ca.crt)
pem_to_c(${KEYS_DIR}/device-608a.crt DEVCERT_PEM_C)   # tls-608a: device cert = 608A public key (CA-signed); private key in chip
# Placeholder private key — ALWAYS the throwaway below, never a real key. altcp's
# 2wayauth config just needs SOME parsable P-256 key (mbedTLS 2.16 doesn't check the
# cert/key pairing; real TLS signing goes to the 608A via ECDSA_SIGN_ALT, d is unused).
# It used to be the ops key device-0001.key, which put a REAL CA-signed identity's
# private key into every flash image (dumpable pre-RDP2) — removed 2026-07-24.
# This key is public-by-design: it matches no certificate and authorizes nothing.
set(DEVKEY_PEM_C "\"-----BEGIN EC PRIVATE KEY-----\\n\"\n\"MHcCAQEEIJIia5qpsE0U27/103GuTvSv0F2TlRPB5zOjdwKg3O7loAoGCCqGSM49\\n\"\n\"AwEHoUQDQgAEg42NbKSkOaCYF0uVBf3OXee6Q9ODUHDfp0Ox3L/b/O/BAGwImzLV\\n\"\n\"3tV0+k5/SLo4fQwizdngaANmmqzS1FcfrA==\\n\"\n\"-----END EC PRIVATE KEY-----\\n\"")
# universal PRODUCTION image: -DAPP_IDENTITY_EMBED=0 embeds NO device certificate — every
# board runs this same hex, identity comes from the littlefs partition written by the
# provisioning jig (tools/provision.ps1, app_identity.c). Default (unset/1) keeps the
# dev behavior of embedding device-608a.crt as the fallback identity.
if(DEFINED APP_IDENTITY_EMBED AND NOT APP_IDENTITY_EMBED)
    set(DEVCERT_PEM_C "\"\"")
    message(STATUS "APP_IDENTITY_EMBED=0: universal image, device identity from littlefs only")
endif()
pem_to_c_first(FWSIGN_PUB_C ${CMAKE_CURRENT_SOURCE_DIR}/../../../keys/fwsign/fwsign.pub ${CERTS_DIR}/fwsign.pub)
pem_to_c(${CMAKE_CURRENT_SOURCE_DIR}/../../../keys/anticlone/dev_se.pub ANTICLONE_PUB_C)
pem_to_c(${CMAKE_CURRENT_SOURCE_DIR}/../../../keys/anticlone/dev_se.key SE_DEV_KEY_C)
pem_to_c(${CMAKE_CURRENT_SOURCE_DIR}/../../../keys/aws/AmazonRootCA1.pem AWS_CA_C)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/App/app_certs.c.in
               ${CMAKE_CURRENT_BINARY_DIR}/app_certs.c @ONLY)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MBEDTLS_SRC}
    ${ALTCP_TLS_DIR}/altcp_tls_mbedtls.c
    ${ALTCP_TLS_DIR}/altcp_tls_mbedtls_mem.c
    ${CMAKE_CURRENT_BINARY_DIR}/app_certs.c
)

# cryptography library always uses -O2 (even in Debug builds): -O0 software elliptic curve takes 20s+ per handshake,
# would be killed by the connection-abort watchdog (20s) => never connects (2026-07-04 measured: TCP reaches broker but handshake never completes)
set_source_files_properties(${MBEDTLS_SRC}
    ${ALTCP_TLS_DIR}/altcp_tls_mbedtls.c
    ${ALTCP_TLS_DIR}/altcp_tls_mbedtls_mem.c
    PROPERTIES COMPILE_OPTIONS "-O2")
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${MBEDTLS_DIR}/include
    ${ALTCP_TLS_DIR}
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    MBEDTLS_CONFIG_FILE="app_mbedtls_config.h"
)

# LwIP is a standalone OBJECT library target (mx-generated.cmake), its sources include altcp_tls.h -> also needs
# mbedTLS header path + config macro + App directory (to find app_mbedtls_config.h)
target_include_directories(LwIP PRIVATE
    ${MBEDTLS_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/App
)
target_compile_definitions(LwIP PRIVATE
    MBEDTLS_CONFIG_FILE="app_mbedtls_config.h"
)

# ---- OTA trial-period test hook (2026-07-07, auto-rollback verification matrix #4/#5; unrelated to TLS, lodged in this user cmake file) ----
# usage: cmake -B build -DOTA_TEST=HANG|NONET; restore production build: cmake -B build -UOTA_TEST (cache remembers it!)
# test builds automatically shorten trial timeout 10min->120s; app_init.c tags the version string with TEST-*, visible in heartbeat/dash to prevent spoofing
if(DEFINED OTA_TEST AND NOT OTA_TEST STREQUAL "")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        OTA_TEST_${OTA_TEST}
        OTA_TRIAL_TIMEOUT_MS=120000)
    message(WARNING "OTA_TEST=${OTA_TEST}: TEST firmware! trial timeout 120s, version tagged TEST-${OTA_TEST}")
endif()

# ---- Modbus RTU frame core (pure C99, no HAL, 757 P1 restored: app_mqtt dependency; contract=docs/Backplane_Bus_Protocol.md) ----
set(MODBUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../modbus)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MODBUS_DIR}/modbus_core.c
    ${MODBUS_DIR}/modbus_slave.c
    ${MODBUS_DIR}/modbus_selftest.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MODBUS_DIR})

# ---- SNTP (2026-07-08 time service): sntp.c copied from FW_H7_V1.13.0, attached to LwIP target ----
target_sources(LwIP PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/Third_Party/LwIP/src/apps/sntp/sntp.c)

# [757 P1 trim] FDCAN block removed entirely, copy back from nucleo app_tls.cmake during P2/P3 port

# ---- OpenAMP/RPMsg (P2① integrated 2026-07-17; guide=Common/openamp/Integration_Guide.md) ----
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
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${OAMP}/open-amp/lib/include ${OAMP}/libmetal/lib/include ${OAMP_GLUE})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    METAL_INTERNAL METAL_MAX_DEVICE_REGIONS=2 NO_ATOMIC_64_SUPPORT
    RPMSG_BUFFER_SIZE=512 VIRTIO_MASTER_ONLY)

# [757 P1 trim] generic UART low-level block removed entirely, copy back from nucleo app_tls.cmake during P2/P3 port
