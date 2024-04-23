# RegisteredCommand
set(TARGET_NAME test_${PROJECT_NAME}_RegisteredCommand)
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


# SimCommand
set(TARGET_NAME test_${PROJECT_NAME}_SimCommand)
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
        PRIVATE
        SimCommand.test.cpp
        ../main/SimCommand.cpp
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
