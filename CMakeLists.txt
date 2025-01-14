cmake_minimum_required(VERSION 3.9)
project(rlss)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party/splx)
add_subdirectory(third_party/lp_wrappers)
find_package(Boost COMPONENTS filesystem REQUIRED)

add_library(rlss INTERFACE)
target_include_directories(
    rlss INTERFACE
    include
    ${Boost_INCLUDE_DIRS}
    third_party/libMultiRobotPlanning/include
)

target_link_libraries(
    rlss INTERFACE
    splx
    qp_wrappers
    lp_wrappers
    ${Boost_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    include(tests/Tests.cmake)
endif()

option(BUILD_RLSS_EXAMPLES "build examples of rlss" ON)
option(ENABLE_RLSS_DEBUG_MESSAGES "enables debug message for rlss" OFF)
option(ENABLE_RLSS_MATHEMATICA_OUTPUT "enables debug message for rlss" OFF)
option(ENABLE_RLSS_JSON_BUILDER "enables json builder for vis tool" ON)
option(ENABLE_RLSS_STATISTICS "enables statistics for rlss" OFF)

if(ENABLE_RLSS_DEBUG_MESSAGES)
    target_compile_definitions(
            rlss
            INTERFACE
            ENABLE_RLSS_DEBUG_MESSAGES
    )
endif()

if(ENABLE_RLSS_MATHEMATICA_OUTPUT)
    target_compile_definitions(
        rlss
        INTERFACE
        ENABLE_RLSS_MATHEMATICA_OUTPUT
    )
endif()

if(ENABLE_RLSS_JSON_BUILDER)
    target_compile_definitions(
        rlss
        INTERFACE
        ENABLE_RLSS_JSON_BUILDER
    )
endif()

if(ENABLE_RLSS_STATISTICS)
    target_compile_definitions(
        rlss
        INTERFACE
        ENABLE_RLSS_STATISTICS
    )
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_RLSS_EXAMPLES)
    include(examples/Examples.cmake)
endif()
