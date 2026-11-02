# ESP-IDF component (Arduino ignores this file and builds src/ itself).
# Each supported chip has a port in src/hardware/<chip>/.
set(port "src/hardware/${IDF_TARGET}")
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/${port}/port.c")
    message(FATAL_ERROR "espsdrtx: no port for ${IDF_TARGET} (see src/hardware/README.md)")
endif()
idf_component_register(SRCS "src/espsdrtx.c" "src/espsdrtx_mod.c"
                            "src/espsdrtx_tables.c" "${port}/port.c"
                       INCLUDE_DIRS "src"
                       REQUIRES esp_wifi nvs_flash esp_event
                       PRIV_REQUIRES esp_phy heap)
# A port may add a linker script, e.g. to fail the link instead of booting
# with static data in the RF bank.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${port}/rf_bank_guard.ld")
    target_link_libraries(${COMPONENT_LIB} INTERFACE
                          "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/${port}/rf_bank_guard.ld")
endif()
