# ESP-IDF component (Arduino ignores this file and builds src/ itself).
# Each supported chip has a port in src/hardware/<chip>/, named as ESP-IDF
# names the target; nothing here names a chip.
set(port "${CMAKE_CURRENT_LIST_DIR}/src/hardware/${IDF_TARGET}")
if(NOT EXISTS "${port}/port.c")
    message(FATAL_ERROR "espsdrtx: no port for ${IDF_TARGET} (see src/hardware/README.md)")
endif()

# Every .c in the port's directory, so a port may split its code.
file(GLOB port_srcs "${port}/*.c")
set(espsdrtx_srcs "src/espsdrtx.c" "src/espsdrtx_mod.c" "src/espsdrtx_tables.c"
                  ${port_srcs})
set(espsdrtx_requires esp_wifi nvs_flash esp_event)
set(espsdrtx_priv_requires esp_phy heap)

# A port may add a port.cmake, included here, which may append to
# espsdrtx_srcs, espsdrtx_requires and espsdrtx_priv_requires (e.g. the
# component that powers the PHY on a chip without Wi-Fi). It is also
# included during ESP-IDF's requirements scan, so it must only set variables.
if(EXISTS "${port}/port.cmake")
    include("${port}/port.cmake")
endif()

idf_component_register(SRCS ${espsdrtx_srcs}
                       INCLUDE_DIRS "src"
                       REQUIRES ${espsdrtx_requires}
                       PRIV_REQUIRES ${espsdrtx_priv_requires})

# A port may add a linker script, e.g. to fail the link instead of booting
# with static data in the RF bank.
if(EXISTS "${port}/rf_bank_guard.ld")
    target_link_libraries(${COMPONENT_LIB} INTERFACE
                          "-Wl,-T${port}/rf_bank_guard.ld")
endif()
