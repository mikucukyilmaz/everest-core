set(TARGET_NAME ${PROJECT_NAME}_module_ev_manager_tests)
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
        PRIVATE
        RegisteredCommand.test.cpp
)

target_link_libraries(${TARGET_NAME}
        PRIVATE
        everest::framework
        everest::log
        Catch2::Catch2WithMain
)
if (NOT DISABLE_EDM)
    list(APPEND CMAKE_MODULE_PATH ${CPM_PACKAGE_catch2_SOURCE_DIR}/extras)
    include(Catch)
    catch_discover_tests(${TARGET_NAME})
endif ()

catch_discover_tests(${TARGET_NAME})
