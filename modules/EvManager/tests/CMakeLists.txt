set(TARGET_NAME test_${PROJECT_NAME}_EvManager)
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
        PRIVATE
        RegisteredCommand.test.cpp
        SimCommand.test.cpp
        ../main/SimCommand.cpp
        SimData.test.cpp
        ../main/SimData.cpp
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)
target_include_directories(${TARGET_NAME} PRIVATE
        ${GENERATED_INCLUDE_DIR}
        ${CMAKE_BINARY_DIR}/generated/modules/${MODULE_NAME}
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
else ()
    add_test(${TARGET_NAME} ${TARGET_NAME})
endif ()
