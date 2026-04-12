cmake_minimum_required(VERSION 3.16)
project(SentinelNetSim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -fsanitize=address,undefined)
    add_link_options(-fsanitize=address,undefined)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/bft_consensus/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/anomaly/include
)

add_executable(sentinelnet_sim
    main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/bft_consensus/bft_consensus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/anomaly/anomaly.c
)

target_link_libraries(sentinelnet_sim m)

enable_testing()
add_test(NAME sentinelnet_sim COMMAND sentinelnet_sim)
