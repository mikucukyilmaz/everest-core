generate_config_run_script(CONFIG sil)
generate_config_run_script(CONFIG sil-two-evse)
generate_config_run_script(CONFIG sil-ocpp)
generate_config_run_script(CONFIG sil-ocpp201)
generate_config_run_script(CONFIG sil-dc)
generate_config_run_script(CONFIG sil-two-evse-dc)
generate_config_run_script(CONFIG sil-energy-management)
generate_config_run_script(CONFIG hil)
generate_config_run_script(CONFIG sil-gen-pm)
generate_config_run_script(CONFIG sil-ocpp)
generate_config_run_script(CONFIG sil-ocpp-custom-extension)

# install configs
install(
    DIRECTORY "."
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/everest"
    FILES_MATCHING PATTERN "*.yaml"
)

# install certificates
install(
    DIRECTORY "certs"
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/everest"
    FILES_MATCHING PATTERN "*.pem" PATTERN "*.key" PATTERN "*.der" PATTERN "*.txt" PATTERN "*.jks" PATTERN "*.p12" 
)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(EVEREST_DEFAULT_LOGGING_FILE "${EVEREST_CONFIG_ASSET_DIR}/debug_logging.ini")
else()
    set(EVEREST_DEFAULT_LOGGING_FILE "${EVEREST_CONFIG_ASSET_DIR}/default_logging.ini")
endif()
install(
    FILES "${EVEREST_DEFAULT_LOGGING_FILE}"
    DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}/everest"
    RENAME "default_logging.cfg"
)

add_subdirectory(nodered)
